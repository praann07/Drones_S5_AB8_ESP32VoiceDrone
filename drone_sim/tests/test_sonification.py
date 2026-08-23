"""TDD tests for sonification: synthesis, WAV export, playback, GCS boundary integration."""
import math
import os
import wave

import pytest


# ---------------------------------------------------------------------------
# 1. Audio synthesis — turn audio params into raw samples
# ---------------------------------------------------------------------------

class TestSynthesizeTone:
    def test_sine_tone_produces_correct_sample_count(self):
        from sonification import synthesize_tone

        samples = synthesize_tone(frequency=440.0, amplitude=0.5, duration=0.1,
                                  waveform="sine", sample_rate=1000)
        assert len(samples) == 100

    def test_sine_tone_samples_stay_within_amplitude(self):
        from sonification import synthesize_tone

        samples = synthesize_tone(frequency=440.0, amplitude=0.5, duration=0.1,
                                  waveform="sine", sample_rate=8000)
        assert all(-0.5 <= s <= 0.5 for s in samples)

    def test_sine_tone_oscillates(self):
        from sonification import synthesize_tone

        samples = synthesize_tone(frequency=1.0, amplitude=1.0, duration=1.0,
                                  waveform="sine", sample_rate=8)
        # One full cycle of a 1 Hz tone at 8 samples/sec crosses zero
        crossings = sum(1 for i in range(1, len(samples))
                        if samples[i - 1] * samples[i] < 0)
        assert crossings >= 1

    def test_square_waveform_only_hits_extremes(self):
        from sonification import synthesize_tone

        samples = synthesize_tone(frequency=10.0, amplitude=0.7, duration=0.1,
                                  waveform="square", sample_rate=8000)
        unique = {round(s, 6) for s in samples}
        assert unique <= {-0.7, 0.7}

    def test_unknown_waveform_raises(self):
        from sonification import synthesize_tone

        with pytest.raises(ValueError):
            synthesize_tone(frequency=440.0, amplitude=1.0, duration=0.01,
                            waveform="banjo", sample_rate=8000)

    def test_amplitude_clamped_to_unit_range(self):
        from sonification import synthesize_tone

        samples = synthesize_tone(frequency=440.0, amplitude=5.0, duration=0.01,
                                  waveform="sine", sample_rate=8000)
        assert all(s <= 1.0 for s in samples)


# ---------------------------------------------------------------------------
# 2. Composition builder — SINDy error dicts -> audio samples
# ---------------------------------------------------------------------------

class TestBuildFlightComposition:
    def _error(self, total=0.2, alt_div=0.1, thrust=0.6, r=0.05):
        return {
            "total": total,
            "qr_coupling": 0.02,
            "rp_coupling": 0.01,
            "pq_coupling": 0.01,
            "altitude_divergence": alt_div,
            "thrust_coupling": thrust,
            "actual_p": 0.01,
            "actual_q": 0.02,
            "actual_r": r,
            "thrust_norm": thrust,
            "est_alt": 5.0,
        }

    def test_composition_returns_sample_list(self):
        from sonification import build_flight_composition

        samples = build_flight_composition([self._error()], points_per_step=50,
                                           sample_rate=1000)
        assert len(samples) == 50

    def test_composition_length_matches_input_count(self):
        from sonification import build_flight_composition

        errors = [self._error() for _ in range(10)]
        samples = build_flight_composition(errors, points_per_step=25,
                                           sample_rate=1000)
        assert len(samples) == 250

    def test_boundary_error_is_louder_than_stable_error(self):
        from sonification import build_flight_composition

        stable = build_flight_composition([self._error(total=0.2)],
                                          points_per_step=200, sample_rate=4000)
        boundary = build_flight_composition(
            [self._error(total=1.8, alt_div=1.5)], points_per_step=200,
            sample_rate=4000)
        peak = lambda xs: max(abs(s) for s in xs)
        assert peak(boundary) > peak(stable)


# ---------------------------------------------------------------------------
# 3. WAV export — write samples to a real, readable .wav file
# ---------------------------------------------------------------------------

class TestExportWav:
    def test_writes_readable_wav_file(self, tmp_path):
        from sonification import export_wav, synthesize_tone

        samples = synthesize_tone(440.0, 0.5, 0.05, "sine", 8000)
        path = tmp_path / "flight.wav"
        export_wav(samples, str(path))

        with wave.open(str(path), "rb") as w:
            assert w.getnchannels() == 1          # mono
            assert w.getsampwidth() == 2          # 16-bit
            assert w.getframerate() == 8000
            assert w.getnframes() == len(samples)

    def test_written_samples_round_trip_within_16bit_precision(self, tmp_path):
        from sonification import export_wav, synthesize_tone

        original = synthesize_tone(440.0, 0.9, 0.02, "sine", 8000)
        path = tmp_path / "round.wav"
        export_wav(original, str(path))

        with wave.open(str(path), "rb") as w:
            raw = w.readframes(w.getnframes())
        recovered = [int.from_bytes(raw[i:i + 2], "little", signed=True) / 32768.0
                     for i in range(0, len(raw), 2)]
        assert len(recovered) == len(original)
        for a, b in zip(recovered, original):
            assert abs(a - b) < 1e-4

    def test_empty_samples_raises_value_error(self, tmp_path):
        from sonification import export_wav

        with pytest.raises(ValueError):
            export_wav([], str(tmp_path / "empty.wav"))


# ---------------------------------------------------------------------------
# 4. Playback — play a wav file without crashing (Windows winsound)
# ---------------------------------------------------------------------------

class TestPlayWav:
    def test_plays_real_file_without_error(self, tmp_path):
        from sonification import export_wav, play_wav, synthesize_tone

        samples = synthesize_tone(440.0, 0.5, 0.05, "sine", 8000)
        path = tmp_path / "tiny.wav"
        export_wav(samples, str(path))
        # Should not raise; returns True on success
        assert play_wav(str(path)) is True

    def test_missing_file_returns_false(self):
        from sonification import play_wav

        assert play_wav("Z:/definitely/not/here.wav") is False


# ---------------------------------------------------------------------------
# 5. GCS integration — boundary status exposed to dashboard
# ---------------------------------------------------------------------------

class TestGCSBoundaryData:
    def test_gcs_data_exposes_sindy_boundary_status(self):
        from main import Drone, GCS
        import config

        d = Drone()
        gcs = GCS(d)
        d.arm()
        gcs.home_pos = (0.0, 0.0)
        for _ in range(20):
            d.step(config.DT)
            gcs.update(config.DT)
            gcs.controller.update(d, config.DT)

        data = gcs.gcs_data()
        assert "sindy" in data
        sindy = data["sindy"]
        assert "is_boundary" in sindy
        assert "error" in sindy
        assert isinstance(sindy["error"], float)

    def test_boundary_flag_matches_error_threshold(self):
        from main import Drone, GCS
        import config

        d = Drone()
        gcs = GCS(d)
        d.arm()
        gcs.home_pos = (0.0, 0.0)
        for _ in range(20):
            d.step(config.DT)
            gcs.update(config.DT)
            gcs.controller.update(d, config.DT)

        sindy = gcs.gcs_data()["sindy"]
        expected = sindy["error"] > config.SINDY_BOUNDARY_THRESHOLD
        assert sindy["is_boundary"] == expected

    def test_landing_exports_flight_composition_wav(self, tmp_path, monkeypatch):
        from main import Drone, GCS
        import config

        out_dir = str(tmp_path)
        monkeypatch.setattr(config, "SONIFICATION_OUTPUT_DIR", out_dir)

        d = Drone()
        gcs = GCS(d)
        gcs.failsafe_enabled = False
        d.arm()
        gcs.home_pos = (0.0, 0.0)
        # Fly briefly then land to trigger the save hook
        gcs.handle("takeoff", {"alt": 3.0})
        for _ in range(300):
            d.step(config.DT)
            gcs.update(config.DT)
            gcs.controller.update(d, config.DT)
        gcs.handle("land", {})
        for _ in range(600):
            d.step(config.DT)
            gcs.update(config.DT)
            gcs.controller.update(d, config.DT)

        wavs = [f for f in os.listdir(out_dir) if f.endswith(".wav")]
        assert len(wavs) == 1
