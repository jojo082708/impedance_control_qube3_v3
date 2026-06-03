"""
gui/widgets.py — 可重用的 GUI 元件。
"""
import tkinter as tk


class ParamWidget(tk.Frame):
    """
    滑桿 + 數值輸入框 的組合元件。
    支援雙向同步：拖滑桿 → 更新輸入框，手動輸入 → 更新滑桿。
    """
    BG       = "#0b0f14"
    ENTRY_BG = "#060a0f"
    ACCENT   = "#00c8ff"
    DANGER   = "#ff4d6a"
    SUBTEXT  = "#7a8899"
    TEXT     = "#dde6f0"
    BORDER   = "#1e2a38"

    def __init__(self, parent, label: str, var: tk.DoubleVar,
                 from_: float, to: float,
                 resolution: float = 0.001, unit: str = "",
                 fmt: str = "{:.3f}", slider_length: int = 110, **kw):
        super().__init__(parent, bg=self.BG, **kw)
        self._var      = var
        self._from     = from_
        self._to       = to
        self._res      = resolution
        self._fmt      = fmt
        self._updating = False

        tk.Label(self, text=label, font=("Consolas", 9),
                 bg=self.BG, fg=self.SUBTEXT,
                 width=22, anchor="w").pack(side="left")

        self._slider = tk.Scale(
            self, variable=var, from_=from_, to=to,
            resolution=resolution, orient="horizontal",
            showvalue=False, bg=self.BG, fg=self.TEXT,
            troughcolor="#060a0f", highlightthickness=0,
            sliderrelief="flat", activebackground=self.ACCENT,
            command=self._on_slider, length=slider_length)
        self._slider.pack(side="left", padx=(4, 6))

        self._entry_var = tk.StringVar(value=fmt.format(var.get()))
        self._entry = tk.Entry(
            self, textvariable=self._entry_var, width=9,
            font=("Consolas", 10), bg=self.ENTRY_BG, fg=self.TEXT,
            insertbackground=self.TEXT, relief="flat",
            highlightthickness=1,
            highlightbackground=self.BORDER,
            highlightcolor=self.ACCENT)
        self._entry.pack(side="left")

        for ev in ("<Return>", "<KP_Enter>", "<FocusOut>", "<Tab>"):
            self._entry.bind(ev, self._on_entry_commit)

        if unit:
            tk.Label(self, text=unit, font=("Consolas", 9),
                     bg=self.BG, fg=self.SUBTEXT,
                     width=7, anchor="w").pack(side="left", padx=(4, 0))

    def _on_slider(self, val):
        if self._updating:
            return
        self._updating = True
        self._entry_var.set(self._fmt.format(float(val)))
        self._flash_normal()
        self._updating = False

    def _on_entry_commit(self, event=None):
        if self._updating:
            return
        try:
            value = float(self._entry_var.get().strip())
        except ValueError:
            self._flash_error()
            self._entry_var.set(self._fmt.format(self._var.get()))
            return
        clamped = max(self._from, min(self._to, value))
        self._updating = True
        self._var.set(round(clamped / self._res) * self._res)
        self._entry_var.set(self._fmt.format(clamped))
        self._flash_normal()
        self._updating = False

    def _flash_error(self):
        self._entry.config(highlightbackground=self.DANGER,
                           highlightcolor=self.DANGER)
        self.after(800, self._flash_normal)

    def _flash_normal(self):
        self._entry.config(highlightbackground=self.BORDER,
                           highlightcolor=self.ACCENT)

    def get(self) -> float:
        return self._var.get()
