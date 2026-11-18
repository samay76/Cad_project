"""Stand-in job for the server tests: python -m tests.sleep_job <seconds> --status <status.json>."""
import json
import signal
import sys
import time

status = sys.argv[sys.argv.index('--status') + 1]


def write(state, message):
    with open(status, 'w') as f:
        json.dump({'state': state, 'message': message}, f)


def on_term(*_):
    write('failed', 'stopped')
    sys.exit(3)


signal.signal(signal.SIGTERM, on_term)
write('running', 'sleeping')
time.sleep(float(sys.argv[1]))
write('done', 'slept')
