#!/usr/bin/env bash
# Restore the dedicated Orin SSH route in a new or existing CAT323 container.
set -euo pipefail
source_dir=/workspaces/gravis_ws/.ssh-agent
install -d -m 700 /root/.ssh
install -m 600 "$source_dir/ssh_config" /root/.ssh/cat323_orin_a.conf
install -m 600 "$source_dir/known_hosts" /root/.ssh/cat323_orin_a_known_hosts
python3 - <<'PY'
import os
from pathlib import Path
import tempfile

config = Path('/root/.ssh/config')
original = config.read_text() if config.exists() else ''
# Migrate the exact block installed for the earlier laptop-agent tunnel.
old_block = '''Host orin-a 10.27.0.10 rk-2609-507835-orin-a
    HostName 10.27.0.10
    User nvidia
    IdentityAgent /workspaces/gravis_ws/.ssh-agent/agent.sock
    StrictHostKeyChecking yes
'''
content = original.replace(old_block, '', 1)
include = 'Include /root/.ssh/cat323_orin_a.conf'
# This include must precede broad Host defaults because SSH uses the first value.
content = '\n'.join(line for line in content.split('\n') if line != include)
content = include + '\n' + content
if content != original:
    descriptor, staged = tempfile.mkstemp(prefix='.cat323-config-', dir=config.parent)
    try:
        with os.fdopen(descriptor, 'w') as stream:
            stream.write(content)
        os.replace(staged, config)
    finally:
        if os.path.exists(staged):
            os.unlink(staged)
PY
