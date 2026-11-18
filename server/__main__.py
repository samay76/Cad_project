"""Starts the pick cell server:  python -m server   (settings: server/settings.py, .env)."""
import uvicorn

from .app import create_app
from .settings import load_settings


def main():
    s = load_settings()
    uvicorn.run(create_app(s), host=s.host, port=s.port, log_level='info')


if __name__ == '__main__':
    main()
