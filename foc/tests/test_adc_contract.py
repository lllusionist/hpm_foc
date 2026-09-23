import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
ADC = (ROOT / "core/inner/src/adc.c").read_text(encoding="utf-8")
FOC = (ROOT / "core/outer/src/foc.c").read_text(encoding="utf-8")
TASK = (ROOT / "core/outer/src/foc_task.c").read_text(encoding="utf-8")


class AdcContractTests(unittest.TestCase):
    def test_dma_layout_and_trigger_isolation(self):
        self.assertRegex(ADC, r"ADC1_TRIG_CH\s+ADC16_CONFIG_TRG0B")
        self.assertEqual(ADC.count("[ADC_SOC_PMT_MAX_DMA_BUFF_LEN_IN_4BYTES]"), 2)
        self.assertEqual(ADC.count("ATTR_PLACE_AT_NONCACHEABLE_BSS_WITH_ALIGNMENT"), 2)
        self.assertIn("ADC1_TRIG_CH * 4U", ADC)

    def test_both_frames_are_validated(self):
        self.assertIn("TRG_CMPT_FLAG", ADC)
        self.assertIn(".seq_num", ADC)
        self.assertIn(".trig_ch", ADC)
        self.assertIn(".adc_ch", ADC)
        self.assertIn("bad_current_frames", ADC)
        self.assertIn("ADC16_INT_STS_AHB_ERR_MASK", ADC)
        self.assertIn("ADC16_INT_STS_DMA_FIFO_FULL_MASK", ADC)
        self.assertNotIn("sample->cycle_bit", ADC)

    def test_ibus_uses_isolated_low_rate_queue(self):
        self.assertRegex(ADC, r"#define ADC_TRIG_LEN \(3U\)")
        self.assertRegex(ADC, r"#define ADC1_IBUS_TRIG_CH ADC16_CONFIG_TRG0C")
        self.assertIn("adc1_read_ibus", ADC)
        self.assertIn("ADC1_IBUS_DMA_OFFSET", ADC)
        self.assertIn("adc1_ibus_ch_tab", ADC)
        self.assertIn("HPM_ADC0->TRG_CMPT_FLAG = (1U << ADC_TRIG_CH)", ADC)
        self.assertNotIn("HPM_ADC0->TRG_CMPT_FLAG = complete", ADC)
        # 0C 无 INTEN，TRG_CMPT_FLAG 不置位，必须靠 DMA 元数据判断完成
        self.assertIn("sample->adc_ch == ADC_CH_IBUS", ADC)
        self.assertIn("sample->trig_ch == ADC1_IBUS_TRIG_CH", ADC)
        self.assertNotIn("adc16_trigger_pmt_by_sw(HPM_ADC0", ADC)
        self.assertIn("FOC.Ibus = ibus_raw", FOC)
        self.assertIn("FOC.Ibus_midpoint", FOC)
        self.assertIn("FOC_IBUS_SCALE", FOC)
        self.assertIn("d[21] = FOC.Ibus_filt", (ROOT / "core/outer/src/vofa.c").read_text(encoding="utf-8"))
        self.assertIn("FOC.adc_ibus_missed >= 3U", TASK)

    def test_sampling_failure_cannot_arm_or_run_indefinitely(self):
        self.assertRegex(FOC, r"bool\s+foc_adc_midpoint\s*\(")
        self.assertRegex(FOC, r"if\s*\(!FOC\.adc_ready\)\s*\{[^}]*return;" )
        self.assertIn("FOC_FAULT_ADC", TASK)
        self.assertIn("FOC.adc_current_missed", TASK)
        self.assertIn("FOC.adc_voltage_missed", TASK)
        self.assertIn("FOC.adc_voltage_missed >= 3U", TASK)
        self.assertIn("FOC.adc_current_frames == current_frames", TASK)
        self.assertIn("!FOC.is_armed", TASK)
        self.assertIn("if (!FOC.adc_ready)", (ROOT / "core/outer/src/foc_cal.c").read_text(encoding="utf-8"))

    def test_midpoint_only_uses_current_adc_samples(self):
        midpoint = FOC[FOC.index("bool foc_adc_midpoint") : FOC.index("void foc_update_voltages")]
        self.assertIn("adc_current_snapshot", midpoint)
        self.assertIn("adc1_read_ibus", midpoint)
        self.assertRegex(midpoint, r"times\s*<\s*16U")
        self.assertRegex(FOC, r"if\s*\(!FOC\.adc_ready\)\s*\{[^}]*return;" )


if __name__ == "__main__":
    unittest.main()
