#!/usr/bin/env python3
"""Drive CSCS authentication in an existing, approved local Chrome session."""
import argparse
import getpass
import json
import os
from pathlib import Path
import re
import shutil
import socket
import subprocess
import tempfile
import time
import urllib.request
from urllib.parse import urlsplit

CDP = Path(__file__).resolve().parents[2] / 'chrome-cdp' / 'scripts' / 'cdp.mjs'
ALLOWED = {'auth.cscs.ch', 'aai-logon.ethz.ch', 'user-account.cscs.ch'}
AUTHENTICATOR_URL = 'chrome-extension://bhghoamapcdpbohphigoooaddinpkbai/view/popup.html'
STATE = r'''JSON.stringify((() => {
  const visible = e => e && e.getClientRects().length > 0;
  const first = selectors => selectors.find(s => visible(document.querySelector(s)));
  return {
    origin: location.origin,
    portalReady: location.origin === 'https://user-account.cscs.ch' && document.title === 'Dashboard - CSCS User Portal',
    device: first(['#device-user-code', 'input[name="device_user_code"]', '#user_code', '#userCode', 'input[name="user_code"]', 'input[name="userCode"]']),
    eth: first(['a[href*="/broker/ETHZ/login"]']),
    username: first(['#username']), password: first(['#password']),
    otp: first(['#otp', 'input[name="otp"]', 'input[autocomplete="one-time-code"]', 'input[name="totp"]']),
    submit: first(['button[type="submit"]', 'input[type="submit"]']),
    failed: /Authentication failed!|Invalid username or password|Invalid authenticator code|Invalid code/i.test(document.body.innerText),
    deviceDone: /You have successfully verified your device|Device authorization successful/i.test(document.body.innerText)
      || (location.pathname === '/auth/realms/cscs/device/status'
          && /You may close this browser window and go back to your device\./i.test(document.body.innerText))
  };
})())'''


def fetch(base, path):
    with urllib.request.urlopen(base + path, timeout=3) as response:
        return json.load(response)


def cdp_call(runtime, target, command, *arguments):
    # Password and OTP travel over the local socket, never process arguments or files.
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as connection:
        connection.settimeout(15)
        connection.connect(str(runtime / f'cdp-{target}.sock'))
        connection.sendall((json.dumps({'id': 1, 'cmd': command, 'args': arguments}) + '\n').encode())
        with connection.makefile('r') as stream:
            result = json.loads(stream.readline())
    if not result.get('ok'):
        raise RuntimeError('Browser action failed; inspect the selected tab manually.')
    return result.get('result', '')


def read_chrome_otp(base, cli, runtime, username):
    pages = [p for p in fetch(base, '/json/list')
             if p.get('type') == 'page' and p.get('url', '').split('#')[0] == AUTHENTICATOR_URL]
    if not pages:
        cli('open', AUTHENTICATOR_URL)
        time.sleep(1)
        pages = [p for p in fetch(base, '/json/list')
                 if p.get('type') == 'page' and p.get('url', '').split('#')[0] == AUTHENTICATOR_URL]
    if not pages:
        raise RuntimeError('Open and unlock Authenticator in Chrome, or use --manual-otp.')
    target = pages[0]['id']
    cli('list')
    cli('eval', target, 'location.origin')  # Start daemon without returning account data.
    expression = f'''(() => {{
      if (location.href.split('#')[0] !== {json.dumps(AUTHENTICATOR_URL)}) return '';
      const entries = Array.from(document.querySelectorAll('#codes .code')).filter(e => {{
        const p = e.parentElement;
        return p.querySelector('.issuer:not(.account)')?.textContent.trim() === 'CSCS'
          && p.querySelector('.account')?.textContent.trim() === {json.dumps(username)};
      }});
      return entries.length === 1 ? entries[0].textContent.replace(/\\s/g, '') : '';
    }})()'''
    # The imported CSCS account uses a 30-second period. Avoid reading at rollover.
    remaining = 30 - time.time() % 30
    if remaining < 5:
        time.sleep(remaining + 1)
    for _ in range(5):
        code = cdp_call(runtime, target, 'eval', expression)
        if re.fullmatch(r'\d{6}', code):
            return code
        time.sleep(1)
    raise RuntimeError('Exactly one unlocked CSCS account was not found in Chrome Authenticator; use --manual-otp if needed.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=9222)
    selection = parser.add_mutually_exclusive_group()
    selection.add_argument('--target', help='CSCS/ETH tab ID or unique prefix')
    selection.add_argument('--url', help='Open and use a CSCS HTTPS authentication URL')
    parser.add_argument('--device-code', help='Initial code printed by cscs-key --headless')
    parser.add_argument('--username', default='lterenzi')
    otp_source = parser.add_mutually_exclusive_group()
    otp_source.add_argument('--manual-otp', action='store_true', help='Prompt for OTP instead of reading Chrome Authenticator')
    otp_source.add_argument('--otp-export', help='Existing Google Authenticator export image; select only CSCS/username')
    parser.add_argument('--inspect', action='store_true', help='Report page state without submitting anything')
    args = parser.parse_args()
    base = f'http://127.0.0.1:{args.port}'
    node = shutil.which('node')
    if not node or not CDP.is_file():
        raise RuntimeError('Node.js and the installed chrome-cdp script are required.')
    ws = urlsplit(fetch(base, '/json/version')['webSocketDebuggerUrl'])
    with tempfile.TemporaryDirectory(prefix='cscs-browser-') as temp:
        port_file = Path(temp) / 'DevToolsActivePort'
        port_file.write_text(f'{ws.port}\n{ws.path}\n')
        env = {**os.environ, 'CDP_PORT_FILE': str(port_file), 'CDP_HOST': '127.0.0.1'}

        def cli(*arguments):
            result = subprocess.run([node, str(CDP), *arguments], env=env,
                                    capture_output=True, text=True, timeout=30)
            if result.returncode:
                raise RuntimeError('Chrome control failed. Check remote debugging and the Chrome Allow prompt.')
            return result.stdout

        selected_target = args.target
        if args.url:
            parsed = urlsplit(args.url)
            if parsed.scheme != 'https' or parsed.hostname not in ALLOWED or parsed.username or parsed.password:
                raise RuntimeError('Only CSCS/ETH HTTPS authentication URLs are accepted.')
            opened = cli('open', args.url)
            match = re.search(r'Opened new tab:\s+([A-Fa-f0-9]+)', opened)
            if not match:
                raise RuntimeError('Could not identify the newly opened tab.')
            selected_target = match.group(1)
            time.sleep(1)
        cli('list')
        pages = [p for p in fetch(base, '/json/list')
                 if p.get('type') == 'page' and urlsplit(p['url']).hostname in ALLOWED]
        if selected_target:
            pages = [p for p in pages if p['id'].lower().startswith(selected_target.lower())]
        if len(pages) != 1:
            raise RuntimeError('Select one CSCS/ETH tab with --target; use chrome-cdp list to see tab IDs.')
        target = pages[0]['id']
        cli('eval', target, 'location.origin')  # Start the selected tab daemon.
        runtime = Path(os.environ.get('XDG_RUNTIME_DIR', str(Path.home() / '.cache'))) / 'cdp'
        def call(command, *arguments):
            return cdp_call(runtime, target, command, *arguments)

        def state():
            return json.loads(call('eval', STATE))

        def fill(selector, value, origin):
            filled = call('eval', f'''(() => {{
              if (location.origin !== {json.dumps(origin)}) return false;
              const e = document.querySelector({json.dumps(selector)});
              if (!(e instanceof HTMLInputElement)) return false;
              const value = {json.dumps(value)};
              Object.getOwnPropertyDescriptor(HTMLInputElement.prototype, 'value').set.call(e, value);
              e.dispatchEvent(new Event('input', {{bubbles: true}}));
              e.dispatchEvent(new Event('change', {{bubbles: true}}));
              return e.value === value;
            }})()''')
            if filled != 'true':
                raise RuntimeError('Page or field changed; stopped before entering credentials.')

        def click(selector, origin):
            result = call('eval', f'''(() => {{
              if (location.origin !== {json.dumps(origin)}) return false;
              const e = document.querySelector({json.dumps(selector)});
              if (!e) return false; e.click(); return true;
            }})()''')
            if result != 'true':
                raise RuntimeError('Page changed before submission.')

        submitted = set()
        for _ in range(120):
            page = state()
            origin = page['origin']
            if args.inspect:
                print(json.dumps(page, indent=2))
                return
            if origin == 'https://user-account.cscs.ch':
                if page['portalReady']:
                    print('Logged in to the CSCS account dashboard.')
                    return
                time.sleep(1)
                continue
            if page['deviceDone'] and origin == 'https://auth.cscs.ch':
                print('CSCS reports successful device authorization.')
                return
            if origin not in {'https://auth.cscs.ch', 'https://aai-logon.ethz.ch'}:
                raise RuntimeError('Unexpected page; finish this step manually in Chrome.')
            if page['failed']:
                raise RuntimeError('The page reports an authentication error. No retry was submitted.')
            if page.get('device') and 'device' not in submitted:
                code = args.device_code or input('Initial CSCS device code: ').strip()
                fill(page['device'], code, origin)
                action = 'device'
            elif page.get('eth') and 'eth' not in submitted:
                click(page['eth'], origin)
                submitted.add('eth')
                time.sleep(1)
                continue
            elif page.get('password') and origin == 'https://aai-logon.ethz.ch' and 'password' not in submitted:
                password = getpass.getpass('ETH web/LDAP password (not saved): ')
                fill(page['username'], args.username, origin)
                fill(page['password'], password, origin)
                del password
                action = 'password'
            elif page.get('otp') and f'otp:{origin}' not in submitted:
                if not args.manual_otp and origin == 'https://auth.cscs.ch':
                    if args.otp_export:
                        from cscs_export_otp import fresh_code, load_secret
                        code = fresh_code(load_secret(args.otp_export, args.username))
                    else:
                        code = read_chrome_otp(base, cli, runtime, args.username)
                else:
                    provider = 'CSCS' if origin == 'https://auth.cscs.ch' else 'ETH'
                    code = getpass.getpass(f'Current {provider} authenticator code (not saved): ').strip()
                if not code.isdigit() or len(code) not in (6, 8):
                    raise RuntimeError('Expected a 6- or 8-digit authenticator code.')
                fill(page['otp'], code, origin)
                del code
                action = f'otp:{origin}'
            else:
                time.sleep(1)
                continue
            if not page.get('submit'):
                raise RuntimeError('No recognized submit button; finish this step in Chrome.')
            click(page['submit'], origin)
            submitted.add(action)
            print(f'Submitted {action.split(":", 1)[0]} step.')
            time.sleep(1)
        raise RuntimeError('Stopped after waiting for the next recognized login step. Check Chrome.')


if __name__ == '__main__':
    try:
        main()
    except (RuntimeError, OSError, ValueError, subprocess.TimeoutExpired) as error:
        raise SystemExit(str(error))
