struct i2c_random_wr_payload start_reg_array_ar0231[] = {{0x301A, 0x91C}};
struct i2c_random_wr_payload stop_reg_array_ar0231[] = {{0x301A, 0x918}};
struct i2c_random_wr_payload start_reg_array_imx390[] = {{0x0, 0}};
struct i2c_random_wr_payload stop_reg_array_imx390[] = {{0x0, 1}};

struct i2c_random_wr_payload init_array_imx390[] = {
  {0x2008, 0xd0}, {0x2009, 0x07}, {0x200a, 0x00}, // MODE_VMAX = time between frames
  {0x200C, 0xe4}, {0x200D, 0x0c},  // MODE_HMAX

  // crop
  {0x3410, 0x88}, {0x3411, 0x7},     // CROP_H_SIZE
  {0x3418, 0xb8}, {0x3419, 0x4},     // CROP_V_SIZE
  {0x0078, 1}, {0x03c0, 1},

  // external trigger (off)
  // while images still come in, they are blank with this
  {0x3650, 0},  // CU_MODE

  // exposure
  {0x000c, 0xc0}, {0x000d, 0x07},
  {0x0010, 0xc0}, {0x0011, 0x07},

  // WUXGA mode
  // not in datasheet, from https://github.com/bogsen/STLinux-Kernel/blob/master/drivers/media/platform/tegra/imx185.c
  {0x0086, 0xc4}, {0x0087, 0xff},   // WND_SHIFT_V = -60
  {0x03c6, 0xc4}, {0x03c7, 0xff},   // SM_WND_SHIFT_V_APL = -60

  {0x201c, 0xe1}, {0x201d, 0x12},   // image read amount
  {0x21ee, 0xc4}, {0x21ef, 0x04},   // image send amount (1220 is the end)
  {0x21f0, 0xc4}, {0x21f1, 0x04},   // image processing amount

  // disable a bunch of errors causing blanking
  {0x0390, 0x00}, {0x0391, 0x00}, {0x0392, 0x00},

  // flip bayer
  {0x2D64, 0x64 + 2},

  // color correction
  {0x0030, 0xf8}, {0x0031, 0x00},  // red gain
  {0x0032, 0x9a}, {0x0033, 0x00},  // gr gain
  {0x0034, 0x9a}, {0x0035, 0x00},  // gb gain
  {0x0036, 0x22}, {0x0037, 0x01},  // blue gain

  // hdr enable (noise with this on for now)
  {0x00f9, 0}
};

struct i2c_random_wr_payload init_array_ar0231[] = {
  {0x301A, 0x0018}, // RESET_REGISTER

  {0x302A, 0x0008}, // VT_PIX_CLK_DIV = 8
  {0x302C, 0x0001}, // VT_SYS_CLK_DIV = 1
  {0x302E, 0x0003}, // PRE_PLL_CLK_DIV = 3
  {0x3030, 0x0064}, // PLL_MULTIPLIER = 100
  {0x3036, 0x000C}, // OP_PIX_CLK_DIV = 12
  {0x3038, 0x0001}, // OP_SYS_CLK_DIV = 1
  {0x30B0, 0x0A00}, // DIGITAL_TEST, bits 0x4000 = 0, DIG_TEST_SUBROW_PAIR_RESET, bits 0x0800 = 1

  // Keep old settings for now? What is different?
  // {0x31B0, 0x004E}, // FRAME_PREAMBLE = 78
  // {0x31B2, 0x0036}, // LINE_PREAMBLE = 54
  // {0x31B4, 0x1866}, // MIPI_TIMING_0 = 6246
  // {0x31B6, 0x2191}, // MIPI_TIMING_1 = 8593
  // {0x31B8, 0x3048}, // MIPI_TIMING_2 = 12360
  // {0x31BA, 0x0187}, // MIPI_TIMING_3 = 391
  // {0x31BC, 0x0006}, // MIPI_TIMING_4 = 6
  // {0x3342, 0x122C}, // MIPI_F1_PDT, bits 0x003F = 44
  // {0x3344, 0x0011}, // MIPI_F1_VC, bits 0x0300 = 0
  // {0x3346, 0x122C}, // MIPI_F2_PDT, bits 0x003F = 44
  // {0x3348, 0x0111}, // MIPI_F2_VC, bits 0x0300 = 1
  // {0x334A, 0x122C}, // MIPI_F3_PDT, bits 0x003F = 44
  // {0x334C, 0x0211}, // MIPI_F3_VC, bits 0x0300 = 2
  // {0x334E, 0x122C}, // MIPI_F4_PDT, bits 0x003F = 44
  // {0x3350, 0x0011}, // MIPI_F4_VC, bits 0x0300 = 0


  {0x3004, 0x0000}, // X_ADDR_START = 0
  {0x3008, 0x0787}, // X_ADDR_END = 1927
  {0x3002, 0x0000}, // Y_ADDR_START = 0
  {0x3006, 0x04B7}, // Y_ADDR_END = 1207
  {0x30A2, 0x0001}, // X_ODD_INC = 1
  {0x30A6, 0x0001}, // Y_ODD_INC = 1
  {0x3402, 0x0788}, // X_OUTPUT_CONTROL = 1928
  {0x3404, 0x04B8}, // Y_OUTPUT_CONTROL = 1208
  {0x3040, 0x0000}, // READ_MODE_VERT_FLIP, 0x8000 = 0, READ_MODE_HORIZ_MIRROR, 0x4000 = 0, READ_MODE_COL_BIN, 0x2000 = 0, READ_MODE_ROW_BIN, 0x1000 = 0
  {0x3082, 0x0008}, // OPERATION_MODE_CTRL_SEQ_CODE, 0x0003 = 0, OPERATION_MODE_CTRL_NUM_EXP, 0x000C = 2, OPERATION_MODE_CTRL_LIM_MODE, 0x0010 = 0, OPERATION_MODE_CTRL_LICM_MODE, 0x0020 = 0, OPERATION_MODE_CTRL_LFM_MODE, 0x0040 = 0
  {0x3044, 0x0400}, // DARK_CONTROL_SHOW_DARK_COLS, 0x0200 = 0, DARK_CONTROL_SHOW_DARK_ROWS, 0x0800 = 0, DARK_CONTROL_SHOW_ATR_ROWS, 0x1000 = 0, DARK_CONTROL_SHOW_DTR_ROWS, 0x2000 = 0
  {0x33E0, 0x0C80}, // TEST_ASIL_ROWS = 3200
  {0x30BA, 0x11E2}, // DIGITAL_CTRL, 0x0003 = 2
  {0x3180, 0x0080}, // DELTA_DK_CONTROL, 0x00F0 = 8
  {0x33E4, 0x0080}, // VERT_SHADING_CTRL, 0x00F0 = 8
  {0x3064, 0xD802}, // SMIA_TEST_EMBED_DATA_EN, 0x0100 = 0, SMIA_TEST_EMBED_NROW, 0x8000 = 1, SMIA_TEST_EMBED_STATS_EN, 0x0080 = 0, SMIA_TEST_STATS_NROW, 0x4000 = 1
  {0x3032, 0x0000}, // SCALING_MODE, 0x0003 = 0
  {0x3400, 0x0010}, // SCALE_M, 0x007F = 16
  {0x306E, 0x9010}, // DATAPATH_SELECT = 36880
  // {0x31D0, 0x0001}, // COMPANDING, 0x0001 = 1
  {0x31D0, 0x0000}, // COMPANDING, 0x0001 = 0
  {0x31AE, 0x0204}, // SERIAL_FORMAT = 516
  // {0x31AC, 0x140C}, // DATA_FORMAT_BITS = 5132
  {0x31AC, 0x0C0C}, // DATA_FORMAT_BITS 12 bit -> 12 bit
  {0x300A, 0x0525}, // FRAME_LENGTH_LINES = 1317
  {0x300C, 0x0BD0}, // LINE_LENGTH_PCK = 3024
  {0x1008, 0x0361}, // FINE_INTEGRATION_TIME_MIN = 865
  {0x100C, 0x0589}, // FINE_INTEGRATION_TIME2_MIN = 1417
  {0x100E, 0x07B1}, // FINE_INTEGRATION_TIME3_MIN = 1969
  {0x1010, 0x0139}, // FINE_INTEGRATION_TIME4_MIN = 313
  {0x3012, 0x0422}, // COARSE_INTEGRATION_TIME = 1058
  {0x3014, 0x0CE2}, // FINE_INTEGRATION_TIME = 3298
  {0x321E, 0x0CE2}, // FINE_INTEGRATION_TIME2 = 3298
  {0x3222, 0x0CE2}, // FINE_INTEGRATION_TIME3 = 3298
  {0x3226, 0x0CAB}, // FINE_INTEGRATION_TIME4 = 3243
  {0x3238, 0x0444}, // EXPOSURE RATIO = 1092
  {0x32EC, 0x72A1}, // SHUT_CTRL2 = 29345
  {0x301A, 0x001C}, // RESET_REGISTER_DRIVE_PINS, 0x0040 = 0, RESET_REGISTER_PARALLEL_EN, 0x0080 = 0, RESET_REGISTER_SER_DIS, 0x1000 = 0, RESET_REGISTER_STREAM, 0x0004 = 1


  // SLAV* MODE
  {0x30CE, 0x0120},
  {0x340A, 0xE6}, // E6 // 0000 1110 0110
  {0x340C, 0x802}, // 2 // 0000 0000 0010

  // Readout Settings
  {0x3342, 0x122C}, // MIPI_F1_PDT_EDT
  {0x3346, 0x122C}, // MIPI_F2_PDT_EDT
  {0x334A, 0x122C}, // MIPI_F3_PDT_EDT
  {0x334E, 0x122C}, // MIPI_F4_PDT_EDT
  {0x3344, 0x0011}, // MIPI_F1_VDT_VC
  {0x3348, 0x0111}, // MIPI_F2_VDT_VC
  {0x334C, 0x0211}, // MIPI_F3_VDT_VC
  {0x3350, 0x0311}, // MIPI_F4_VDT_VC
  {0x31B0, 0x0053}, // FRAME_PREAMBLE
  {0x31B2, 0x003B}, // LINE_PREAMBLE
  {0x301A, 0x001C}, // RESET_REGISTER

  // Noise Corrections
  {0x3092, 0x0C24}, // ROW_NOISE_CONTROL
  {0x337A, 0x0C80}, // DBLC_SCALE0
  {0x3370, 0x03B1}, // DBLC

  // Enable dead pixel correction using
  // the 1D line correction scheme
  {0x31E0, 0x0003},

  // HDR Settings
  {0x33DA, 0x0000}, // COMPANDING
  {0x318E, 0x0200}, // PRE_HDR_GAIN_EN

  // DLO Settings
  {0x3100, 0x4000}, // DLO_CONTROL0
  {0x3280, 0x0CCC}, // T1 G1
  {0x3282, 0x0CCC}, // T1 R
  {0x3284, 0x0CCC}, // T1 B
  {0x3286, 0x0CCC}, // T1 G2
  {0x3288, 0x0FA0}, // T2 G1
  {0x328A, 0x0FA0}, // T2 R
  {0x328C, 0x0FA0}, // T2 B
  {0x328E, 0x0FA0}, // T2 G2

   // Initial Gains
  {0x3022, 0x0001}, // GROUPED_PARAMETER_HOLD_
  {0x3366, 0xFF77}, // ANALOG_GAIN (1x) (A)

  {0x3060, 0x3333}, // ANALOG_COLOR_GAIN

  {0x3362, 0x0000}, // DC GAIN (A & B)

  {0x305A, 0x00F8}, // red gain (A)
  {0x3058, 0x0122}, // blue gain (A)
  {0x3056, 0x009A}, // g1 gain (A)
  {0x305C, 0x009A}, // g2 gain (A)

  {0x3022, 0x0000}, // GROUPED_PARAMETER_HOLD_

  // Initial Integration Time
  {0x3012, 0x0005}, // (A)
};
