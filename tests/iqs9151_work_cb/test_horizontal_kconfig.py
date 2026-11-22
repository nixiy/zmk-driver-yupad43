"""Validate the real horizontal-mode Kconfig choice with kconfiglib."""
from pathlib import Path
from tempfile import TemporaryDirectory

import kconfiglib


def main():
    driver_kconfig = Path(__file__).resolve().parents[2] / "drivers/input/Kconfig"
    with TemporaryDirectory(prefix="iqs9151-kconfig-") as directory:
        root = Path(directory) / "Kconfig"
        root.write_text(
            "".join(
                f"config {symbol}\n    bool\n    default y\n"
                for symbol in ("GPIO", "I2C", "INPUT", "DT_HAS_AZOTEQ_IQS9151_ENABLED")
            ) + "config LOG\n    bool\n" +
            "config INPUT_LOG_LEVEL\n    int\n    default 0\n" +
            f'source "{driver_kconfig.as_posix()}"\n',
            encoding="utf-8",
        )
        config = kconfiglib.Kconfig(str(root))
        assert not config.warnings, config.warnings
        modes = [config.syms[f"INPUT_IQS9151_2F_HORIZONTAL_{mode}"]
                 for mode in ("SCROLL", "SWIPE", "DISABLED")]
        choice = config.named_choices["INPUT_IQS9151_2F_HORIZONTAL_MODE"]
        assert choice.selection is modes[0], "Default must remain horizontal scrolling"
        assert config.syms["INPUT_IQS9151_2F_HORIZONTAL_SWIPE_THRESHOLD"].str_value == "200"
        for selected in modes[1:] + modes[:1]:
            selected.set_value("y")
            assert choice.selection is selected
            assert [symbol.str_value for symbol in modes] == [
                "y" if symbol is selected else "n" for symbol in modes
            ], "Horizontal operations must be mutually exclusive"
            assert config.syms["INPUT_IQS9151_SCROLL_Y_ENABLE"].str_value == "y"
        assert "INPUT_IQS9151_SCROLL_X_ENABLE" not in config.syms
        assert "INPUT_IQS9151_2F_HORIZONTAL_SWIPE_ENABLE" not in config.syms
    print("PASS: horizontal mode default, exclusivity, vertical independence, retired flags")


if __name__ == "__main__":
    main()
