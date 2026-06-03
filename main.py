"""
main.py — 程式入口點。

執行方式：
    python main.py
"""
from data.state import SharedState
from gui.app    import ImpedanceControlPanel


def main():
    state = SharedState()
    app   = ImpedanceControlPanel(state)
    app.mainloop()


if __name__ == "__main__":
    main()
