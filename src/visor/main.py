import sys
import os

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from visor.gui.gui_loader import main as load_gui

def main():
    load_gui()

if __name__ == "__main__":
    main()