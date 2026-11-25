"""Ensure behavior input processing receives stack space without affecting unused builds."""
from pathlib import Path
from tempfile import TemporaryDirectory

import kconfiglib


def main():
    module = Path(__file__).resolve().parents[2]
    with TemporaryDirectory(prefix="iqs9151-stack-") as directory:
        root = Path(directory) / "Kconfig"
        root.write_text(
            'config SETTINGS\n    bool\n'
            'config DT_HAS_ZMK_INPUT_PROCESSOR_BEHAVIORS_CONSUME_ENABLED\n'
            '    bool "Test processor presence"\n'
            f'source "{(module / "input_processors/Kconfig").as_posix()}"\n'
            'config INPUT_THREAD_STACK_SIZE\n    int "Input stack"\n    default 512\n',
            encoding="utf-8",
        )
        config = kconfiglib.Kconfig(str(root))
        stack = config.syms["INPUT_THREAD_STACK_SIZE"]
        present = config.syms["DT_HAS_ZMK_INPUT_PROCESSOR_BEHAVIORS_CONSUME_ENABLED"]
        enabled = config.syms["ZMK_INPUT_PROCESSOR_BEHAVIORS_CONSUME"]
        assert stack.str_value == "512", "Unused processor must not increase stack"
        present.set_value("y")
        assert stack.str_value == "2048", "Instantiated processor needs the larger stack"
        enabled.set_value("n")
        assert stack.str_value == "512", "Disabled processor must not increase stack"
        enabled.set_value("y")
        stack.set_value("4096")
        assert stack.str_value == "4096", "Larger user budgets must be preserved"
        assert not config.warnings, config.warnings
    print("PASS: behavior input stack default, instance gating, disabled builds, user override")


if __name__ == "__main__":
    main()
