struct i2c_random_wr_payload start_reg_array[] = {{0x301A, 0x91C}};
struct i2c_random_wr_payload stop_reg_array[] = {{0x301A, 0x918}};

struct i2c_random_wr_payload init_array_ar0231[] = {
  {0x301A, 0x0018}, // RESET_REGISTER

  // CLOCK Settings
  {0x302A, 0x0006}, // VT_PIX_CLK_DIV
  {0x302C, 0x0001}, // VT_SYS_CLK_DIV
  {0x302E, 0x0002}, // PRE_PLL_CLK_DIV
  {0x3030, 0x0032}, // PLL_MULTIPLIER
  {0x3036, 0x000C}, // OP_WORD_CLK_DIV
  {0x3038, 0x0001}, // OP_SYS_CLK_DIV

  // FORMAT
  {0x3040, 0xC000}, // READ_MODE
  {0x3004, 0x0000}, // X_ADDR_START_ (A)
  {0x308A, 0x0000}, // X_ADDR_START_ (B)
  {0x3008, 0x0787}, // X_ADDR_END_ (A)
  {0x308E, 0x0787}, // X_ADDR_END_ (B)
  {0x3002, 0x0000}, // Y_ADDR_START_ (A)
  {0x308C, 0x0000}, // Y_ADDR_START_ (B)
  {0x3006, 0x04B7}, // Y_ADDR_END_ (A)
  {0x3090, 0x04B7}, // Y_ADDR_END_ (B)
  {0x3032, 0x0000}, // SCALING_MODE
  {0x30A2, 0x0001}, // X_ODD_INC_ (A)
  {0x30AE, 0x0001}, // X_ODD_INC_ (B)
  {0x30A6, 0x0001}, // Y_ODD_INC_ (A)
  {0x30A8, 0x0001}, // Y_ODD_INC_ (B)
  {0x3402, 0x0F10}, // X_OUTPUT_CONTROL
  {0x3404, 0x0970}, // Y_OUTPUT_CONTROL
  {0x3064, 0x1802}, // SMIA_TEST
  {0x30BA, 0x11F2}, // DIGITAL_CTRL

  // SLAV* MODE
  {0x30CE, 0x0120},
  {0x340A, 0xE6}, // E6 // 0000 1110 0110
  {0x340C, 0x802}, // 2 // 0000 0000 0010

  // Readout timing
  {0x300C, 0x07B9}, // LINE_LENGTH_PCK (A)
  {0x303E, 0x07B9}, // LINE_LENGTH_PCK (B)
  {0x300A, 0x07E7}, // FRAME_LENGTH_LINES (A)
  {0x30AA, 0x07E7}, // FRAME_LENGTH_LINES (B)
  {0x3042, 0x0000}, // EXTRA_DELAY

  // Readout Settings
  {0x31AE, 0x0204}, // SERIAL_FORMAT, 4-lane MIPI
  {0x31AC, 0x140C}, // DATA_FORMAT_BITS, 20 -> 12
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
  {0x3044, 0x0400}, // DARK_CONTROL

  // Enable dead pixel correction using
  // the 1D line correction scheme
  {0x31E0, 0x0003},

  // HDR Settings
  {0x3082, 0x0008}, // OPERATION_MODE_CTRL (A) (enable HDR + 3 exposures)

  {0x3014, 0x098E}, // FINE_INTEGRATION_TIME_ (A)
  {0x321E, 0x098E}, // FINE_INTEGRATION_TIME2 (A)
  {0x3222, 0x098E}, // FINE_INTEGRATION_TIME3 (A)

  {0x31D0, 0x0001}, // COMPANDING enabled
  {0x33DA, 0x0001}, // OC_LUT_CONTROL - OC_LEGACY_COMPANDING

  // TODO: use more kneepoints
  // {0x33DA, 0x0002}, // OC_LUT_CONTROL - OC_SET_LUT_DEFAULT
  // {0x31AC, 0x140C}, // Write format again to load kneepoints

  {0x3190, 0x0000}, // 
  {0x318E, 0x0200}, // PRE_HDR_GAIN_EN
  {0x319E, 0x0000}, // s12_dlo_range?

  // DLO Settings
  // {0x3100, 0x4000}, // DLO_CONTROL0

  {0x31A2, 0x0000}, // noise_dlo_dis_threshold

  {0x3280, 0x0fff}, // T1 G1
  {0x3282, 0x0fff}, // T1 R
  {0x3284, 0x0fff}, // T1 B
  {0x3286, 0x0fff}, // T1 G2

  {0x3288, 0x0000}, // T2 G1
  {0x328A, 0x0000}, // T2 R
  {0x328C, 0x0000}, // T2 B
  {0x328E, 0x0000}, // T2 G2

  {0x3290, 0x0000}, // T3 G1
  {0x3292, 0x0000}, // T3 R
  {0x3294, 0x0000}, // T3 B
  {0x3296, 0x0000}, // T3 G2

  // {0x3060, 0xAAAA}, // ANALOG_COLOR_GAIN
  {0x3060, 0x3333}, // ANALOG_COLOR_GAIN

  {0x305A, 0x00F8}, // red gain (A)
  {0x3058, 0x0122}, // blue gain (A)
  {0x3056, 0x009A}, // g1 gain (A)
  {0x305C, 0x009A}, // g2 gain (A)

  {0x3012, 500},   // Integration time (A)
  {0x3238, 0x0011}, // EXPOSURE_RATIO (A)
  {0x3366, 0xF17D}, // ANALOG_GAIN (A) 0.25 - 1.0 - 4.0
  {0x3362, 0x0001}, // DC GAIN (A - T1)
};
