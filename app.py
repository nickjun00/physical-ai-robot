
"""Backward-compatible entry point for the Pygame robot simulator."""

from pygame_app import run


if __name__ == "__main__":
    # Keep `python app.py` working, even though pygame_app.py is the main UI.
    run()
