#!/usr/bin/env python3


class color:
    RESET = "\u001b[0m"
    
    # 8 Colors
    BLACK = "\u001b[30m"
    RED = "\u001b[31m"
    GREEN = "\u001b[32m"
    YELLOW = "\u001b[33m"
    BLUE = "\u001b[34m"
    MAGENTA = "\u001b[35m"
    CYAN = "\u001b[36m"
    WHITE = "\u001b[37m"

    # 8 Colors (BG)
    BGBLACK = "\u001b[40m"
    BGRED = "\u001b[41m"
    BGGREEN = "\u001b[42m"
    BGYELLOW = "\u001b[43m"
    BGBLUE = "\u001b[44m"
    BGMAGENTA = "\u001b[45m"
    BGCYAN = "\u001b[46m"
    BGWHITE = "\u001b[47m"

    # 16 Colors (Bright)
    BBLACK = "\u001b[30;1m"
    BRED = "\u001b[31;1m"
    BGREEN = "\u001b[32;1m"
    BYELLOW = "\u001b[33;1m"
    BBLUE = "\u001b[34;1m"
    BMAGENTA = "\u001b[35;1m"
    BCYAN = "\u001b[36;1m"
    BWHITE = "\u001b[37;1m"

    # 16 Colors (Bright BG)
    BBGBLACK = "\u001b[40;1m"
    BBGRED = "\u001b[41;1m"
    BBGGREEN = "\u001b[42;1m"
    BBGYELLOW = "\u001b[43;1m"
    BBGBLUE = "\u001b[44;1m"
    BBGMAGENTA = "\u001b[45;1m"
    BBGCYAN = "\u001b[46;1m"
    BBGWHITE = "\u001b[47;1m"

    # 256 Colors
    def C256(color: int) -> str:
        return f"\u001b[38;5;{color}m"

    # 256 Colors (BG)
    def BG256(color: int) -> str:
        return f"\u001b[48;5;{color}m"
    
    # DECORATIONS
    BOLD = "\u001b[1m"
    UNDERLINE = "\u001b[4m"
    BLINK = "\u001b[7m"

class move:
    def UP(lines: int) -> str:
        return f"\u001b[{lines}A"
    
    def DOWN(lines: int) -> str:
        return f"\u001b[{lines}B"
    
    def RIGHT(chars: int) -> str:
        return f"\u001b[{chars}C"
    
    def LEFT(chars: int) -> str:
        return f"\u001b[{chars}D"
    