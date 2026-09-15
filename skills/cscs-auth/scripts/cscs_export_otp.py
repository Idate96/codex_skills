#!/usr/bin/env python3
"""Validate a CSCS Google export or enter its OTP in an inspected Chrome form.

No mode prints the secret, export payload, page URL, or generated code.
"""
import argparse
import base64
import hashlib
import hmac
import struct
import subprocess
import time
from urllib.parse import parse_qs, urlsplit


def varint(data, offset):
    value = 0
    for shift in range(0, 70, 7):
        if offset >= len(data):
            raise ValueError('Truncated authenticator export.')
        byte = data[offset]
        offset += 1
        value |= (byte & 127) << shift
        if byte < 128:
            return value, offset
    raise ValueError('Invalid authenticator export integer.')


def fields(data):
    offset = 0
    while offset < len(data):
        tag, offset = varint(data, offset)
        number, wire = tag >> 3, tag & 7
        if number == 0:
            raise ValueError('Invalid authenticator export field.')
        if wire == 0:
            value, offset = varint(data, offset)
        elif wire == 2:
            length, offset = varint(data, offset)
            end = offset + length
            if end > len(data):
                raise ValueError('Truncated authenticator export field.')
            value, offset = data[offset:end], end
        else:
            raise ValueError('Unsupported authenticator export field.')
        yield number, value


def select_secret(uri, username='lterenzi'):
    """Select one account in memory; reject unsupported or ambiguous exports."""
    parsed = urlsplit(uri)
    if parsed.scheme != 'otpauth-migration' or parsed.netloc != 'offline':
        raise ValueError('Expected a Google Authenticator migration export.')
    encoded = parse_qs(parsed.query).get('data', [])
    if len(encoded) != 1:
        raise ValueError('Expected one migration payload.')
    try:
        raw = base64.b64decode(encoded[0], validate=True)
    except (ValueError, UnicodeError) as error:
        raise ValueError('Invalid migration encoding.') from error
    records = list(fields(raw))
    # Never select from one part of a multi-QR batch: another part may duplicate it.
    batch_sizes = [v for k, v in records if k == 3]
    if batch_sizes and batch_sizes != [1]:
        raise ValueError('Multi-image exports are unsupported; use a complete single-image export.')
    matches = []
    for number, value in records:
        if number != 1:
            continue
        if not isinstance(value, bytes):
            raise ValueError('Invalid account record.')
        pairs = list(fields(value))
        account = dict(pairs)
        if len(account) != len(pairs):
            raise ValueError('Duplicate fields in account record.')
        if account.get(3) == b'CSCS' and account.get(2) in (
            username.encode(), f'CSCS:{username}'.encode()
        ):
            matches.append(account)
    if len(matches) != 1:
        raise ValueError('Expected exactly one CSCS account matching the requested username.')
    account = matches[0]
    if (account.get(4), account.get(5), account.get(6)) != (1, 1, 2):
        raise ValueError('Only the verified SHA-1, six-digit TOTP format is supported.')
    secret = account.get(1)
    if not isinstance(secret, bytes) or not secret:
        raise ValueError('The selected CSCS account has no secret.')
    return secret


def load_secret(image_path, username='lterenzi'):
    # Lazy imports let the parser/vector tests and --help run without QR packages.
    from PIL import Image
    import zxingcpp

    with Image.open(image_path) as image:
        exports = [item.text for item in zxingcpp.read_barcodes(image)
                   if item.text.startswith('otpauth-migration:')]
    if len(exports) != 1:
        raise ValueError('Expected exactly one Google Authenticator export QR.')
    return select_secret(exports[0], username)


def totp(secret, timestamp):
    digest = hmac.new(secret, struct.pack('>Q', int(timestamp) // 30), hashlib.sha1).digest()
    offset = digest[-1] & 15
    value = struct.unpack('>I', digest[offset:offset + 4])[0] & 0x7fffffff
    return f'{value % 1000000:06d}'


def fresh_code(secret):
    remaining = 30 - time.time() % 30
    if remaining < 8:
        time.sleep(remaining + 1)
    return totp(secret, time.time())


def run(*args, input=None):
    # xclip's clipboard owner can outlive its parent. Do not give that owner
    # captured output pipes that would keep subprocess.run waiting for EOF.
    return subprocess.run(
        args, input=input, stdout=subprocess.PIPE if input is None else subprocess.DEVNULL,
        stderr=subprocess.DEVNULL, check=True, timeout=10,
    ).stdout


def assert_window(window):
    active = int(run('xdotool', 'getactivewindow').strip())
    if active != int(window, 0):
        raise ValueError('The selected Chrome window is not active; inspect it before retrying.')


def assert_cscs_url(url):
    parsed = urlsplit(url.strip())
    if (parsed.scheme != 'https' or parsed.hostname != 'auth.cscs.ch'
            or parsed.port not in (None, 443) or parsed.username or parsed.password
            or not parsed.path.startswith('/auth/realms/cscs/login-actions/')):
        raise ValueError('Expected the CSCS HTTPS login form; no OTP was entered.')


def submit_desktop(secret, window, field, submit):
    """Use only after visual inspection of the current OTP form/coordinates."""
    assert_window(window)
    # Clear first so a failed copy cannot reuse a stale approved URL.
    run('xclip', '-selection', 'clipboard', input=b'')
    try:
        run('xdotool', 'key', '--clearmodifiers', 'ctrl+l', 'ctrl+c')
        assert_cscs_url(run('xclip', '-o', '-selection', 'clipboard').decode())
    finally:
        run('xdotool', 'key', '--clearmodifiers', 'Escape')
        run('xclip', '-selection', 'clipboard', input=b'')
    code = fresh_code(secret)
    assert_window(window)
    run('xdotool', 'mousemove', '--window', window, *map(str, field), 'click', '1')
    assert_window(window)
    run('xdotool', 'key', '--clearmodifiers', 'ctrl+a')
    run('xdotool', 'type', '--clearmodifiers', '--file', '-', input=code.encode())
    assert_window(window)
    run('xdotool', 'mousemove', '--window', window, *map(str, submit), 'click', '1')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--export-image', required=True)
    parser.add_argument('--username', default='lterenzi')
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--inspect', action='store_true', help='Validate only, without printing a code')
    mode.add_argument('--window', help='Currently active, visually inspected Chrome X11 window ID')
    parser.add_argument('--field', type=int, nargs=2, metavar=('X', 'Y'))
    parser.add_argument('--submit', type=int, nargs=2, metavar=('X', 'Y'))
    args = parser.parse_args()
    if args.window and (args.field is None or args.submit is None):
        parser.error('--window requires --field X Y and --submit X Y')
    secret = load_secret(args.export_image, args.username)
    if args.inspect:
        print('Verified one matching CSCS account: TOTP, SHA-1, six digits. No code displayed.')
        return
    submit_desktop(secret, args.window, args.field, args.submit)
    print('Submitted the CSCS OTP once. Verify the browser, pending CLI, certificate, and SSH.')


if __name__ == '__main__':
    try:
        main()
    except (OSError, ValueError, ImportError, subprocess.SubprocessError) as error:
        # Library errors can contain input data or subprocess arguments; keep them private.
        raise SystemExit('Export/OTP step failed. Check the selected export, dependencies, window, and CSCS form.') from None
