import diagnostic_msgs

ROBOCLAW_ERRORS = {
    0x0000: (diagnostic_msgs.msg.DiagnosticStatus.OK, "Normal"),
    0x0001: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M1 over current"),
    0x0002: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M2 over current"),
    0x0004: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Emergency Stop"),
    0x0008: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Temperature1"),
    0x0010: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Temperature2"),
    0x0020: (
        diagnostic_msgs.msg.DiagnosticStatus.ERROR,
        "Main batt voltage high",
    ),
    0x0040: (
        diagnostic_msgs.msg.DiagnosticStatus.ERROR,
        "Logic batt voltage high",
    ),
    0x0080: (
        diagnostic_msgs.msg.DiagnosticStatus.ERROR,
        "Logic batt voltage low",
    ),
    0x0100: (diagnostic_msgs.msg.DiagnosticStatus.WARN,
             "M1 driver fault"),
    0x0200: (diagnostic_msgs.msg.DiagnosticStatus.WARN,
             "M2 driver fault"),
    0x0400: (
        diagnostic_msgs.msg.DiagnosticStatus.WARN,
        "Main batt voltage high",
    ),
    0x0800: (
        diagnostic_msgs.msg.DiagnosticStatus.WARN,
        "Main batt voltage low",
    ),
    0x1000: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Temperature1"),
    0x2000: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Temperature2"),
    0x4000: (diagnostic_msgs.msg.DiagnosticStatus.OK, "M1 home"),
    0x8000: (diagnostic_msgs.msg.DiagnosticStatus.OK, "M2 home"),
}
