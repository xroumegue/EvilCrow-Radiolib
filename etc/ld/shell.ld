SECTIONS
{
    .FREERTOS_SHELL_CMD_SECTION :
    {
        __freertos_shell_cmd_start = ABSOLUTE(.);
        KEEP(*(.FREERTOS_SHELL_CMD_SECTION))
        __freertos_shell_cmd_end = ABSOLUTE(.);
    } > dram0_0_seg
}

