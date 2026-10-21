#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import threading
from datetime import datetime
from math import gcd

import numpy as np
import sounddevice as sd
import soundfile as sf
import noisereduce as nr

from scipy.signal import butter, sosfiltfilt, resample_poly

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from faster_whisper import WhisperModel


class WhisperSTTNode(Node):

    def __init__(self):
        super().__init__('stt_node')

        # =====================================================
        # ROS 2 PARAMETERS
        # =====================================================

        self.declare_parameter('model_size', 'base')
        self.declare_parameter('input_device', -1)
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('noise_reduction', 0.65)
        self.declare_parameter('min_rms', 0.003)
        self.declare_parameter('max_recording_seconds', 60)
        self.declare_parameter(
            'save_directory',
            '/tmp/stt_recordings'
        )

        self.model_size = str(
            self.get_parameter('model_size').value
        )

        device_param = int(
            self.get_parameter('input_device').value
        )

        # -1: micro mac dinh cua he thong
        self.input_device = (
            None if device_param < 0 else device_param
        )

        self.target_sample_rate = int(
            self.get_parameter('sample_rate').value
        )

        self.noise_reduction = float(
            self.get_parameter('noise_reduction').value
        )

        self.min_rms = float(
            self.get_parameter('min_rms').value
        )

        self.max_recording_seconds = int(
            self.get_parameter(
                'max_recording_seconds'
            ).value
        )

        self.save_directory = str(
            self.get_parameter('save_directory').value
        )

        os.makedirs(
            self.save_directory,
            exist_ok=True
        )

        # =====================================================
        # ROS 2 TOPICS
        # =====================================================

        self.prompt_publisher = self.create_publisher(
            String,
            '/ai_prompt',
            10
        )

        self.start_subscriber = self.create_subscription(
            String,
            '/stt_start',
            self.start_recording_callback,
            10
        )

        self.stop_subscriber = self.create_subscription(
            String,
            '/stt_stop',
            self.stop_recording_callback,
            10
        )

        self.response_subscriber = self.create_subscription(
            String,
            '/ai_response',
            self.ai_response_callback,
            10
        )

        # =====================================================
        # RECORDING STATE
        # =====================================================

        self.state_lock = threading.Lock()

        self.audio_frames = []
        self.audio_samples_count = 0

        self.is_recording = False
        self.is_processing = False

        self.audio_stream = None
        self.input_sample_rate = None
        self.input_device_name = ''

        self.audio_status = None

        # =====================================================
        # LOAD WHISPER
        # =====================================================

        self.get_logger().info(
            f'Dang tai Faster-Whisper: {self.model_size}'
        )

        self.model = WhisperModel(
            self.model_size,
            device='cpu',
            compute_type='int8'
        )

        self.get_logger().info(
            'Da tai Faster-Whisper thanh cong.'
        )

        self.print_audio_devices()

        self.get_logger().info(
            'STT Node san sang.'
        )

        self.get_logger().info(
            'Bat dau ghi am: /stt_start'
        )

        self.get_logger().info(
            'Ket thuc ghi am: /stt_stop'
        )

    # =========================================================
    # LIST AUDIO DEVICES
    # =========================================================

    def print_audio_devices(self):

        try:
            devices = sd.query_devices()

            self.get_logger().info(
                'Danh sach micro dau vao:'
            )

            for index, device in enumerate(devices):

                if device['max_input_channels'] > 0:

                    self.get_logger().info(
                        f"[{index}] {device['name']} | "
                        f"Channels={device['max_input_channels']} | "
                        f"Rate={device['default_samplerate']}"
                    )

            self.get_logger().info(
                f'Micro mac dinh: {sd.default.device}'
            )

        except Exception as e:

            self.get_logger().error(
                f'Loi liet ke micro: {e}'
            )

    # =========================================================
    # GET MICRO CONFIGURATION
    # =========================================================

    def get_input_configuration(self):

        info = sd.query_devices(
            self.input_device,
            'input'
        )

        if info['max_input_channels'] < 1:

            raise RuntimeError(
                'Thiet bi khong ho tro ghi am.'
            )

        sample_rate = int(
            round(info['default_samplerate'])
        )

        if sample_rate <= 0:

            raise RuntimeError(
                'Tan so lay mau khong hop le.'
            )

        return sample_rate, info['name']

    # =========================================================
    # START RECORDING
    # =========================================================

    def start_recording_callback(self, msg):

        command = msg.data.strip().lower()

        if command not in ('start', '1'):
            return

        with self.state_lock:

            if self.is_recording:

                self.get_logger().warning(
                    'Micro dang ghi am.'
                )

                return

            if self.is_processing:

                self.get_logger().warning(
                    'Dang xu ly am thanh. Vui long cho.'
                )

                return

            try:

                sample_rate, device_name = (
                    self.get_input_configuration()
                )

                self.input_sample_rate = sample_rate
                self.input_device_name = device_name

                self.audio_frames = []
                self.audio_samples_count = 0
                self.audio_status = None

                self.audio_stream = sd.InputStream(
                    device=self.input_device,
                    samplerate=self.input_sample_rate,
                    channels=1,
                    dtype='float32',
                    blocksize=0,
                    callback=self.audio_callback
                )

                self.is_recording = True

                self.audio_stream.start()

                self.get_logger().info(
                    'BAT DAU GHI AM. Hay noi...'
                )

                self.get_logger().info(
                    f'Micro: {self.input_device_name}'
                )

                self.get_logger().info(
                    f'Sample rate: {self.input_sample_rate} Hz'
                )

            except Exception as e:

                self.is_recording = False

                stream = self.audio_stream
                self.audio_stream = None

                if stream is not None:

                    try:
                        stream.close()
                    except Exception:
                        pass

                self.get_logger().error(
                    f'Khong the bat dau ghi am: {e}'
                )

    # =========================================================
    # AUDIO CALLBACK
    # =========================================================

    def audio_callback(
        self,
        indata,
        frames,
        time_info,
        status
    ):

        with self.state_lock:

            if status:
                self.audio_status = str(status)

            if not self.is_recording:
                return

            # Gioi han do dai ghi am de kiem soat RAM
            max_samples = (
                self.max_recording_seconds
                * self.input_sample_rate
            )

            remaining = (
                max_samples - self.audio_samples_count
            )

            if remaining <= 0:
                return

            count = min(frames, remaining)

            # Luu ban sao, khong giu tham chieu indata
            self.audio_frames.append(
                indata[:count, 0].copy()
            )

            self.audio_samples_count += count

    # =========================================================
    # STOP RECORDING
    # =========================================================

    def stop_recording_callback(self, msg):

        command = msg.data.strip().lower()

        if command not in ('stop', '0'):
            return

        with self.state_lock:

            if not self.is_recording:

                self.get_logger().warning(
                    'Hien tai khong ghi am.'
                )

                return

            # Ngung nhan du lieu moi
            self.is_recording = False

            stream = self.audio_stream
            self.audio_stream = None

        # Dong stream truoc khi lay du lieu cuoi cung
        try:

            if stream is not None:

                stream.stop()
                stream.close()

        except Exception as e:

            self.get_logger().error(
                f'Loi dong micro: {e}'
            )

        with self.state_lock:

            frames = self.audio_frames

            self.audio_frames = []

            self.is_processing = True

            audio_status = self.audio_status
            self.audio_status = None

        if audio_status:

            self.get_logger().warning(
                f'Trang thai audio: {audio_status}'
            )

        if not frames:

            self.get_logger().warning(
                'Khong thu duoc am thanh.'
            )

            self.set_processing_done()
            return

        audio = np.concatenate(
            frames
        ).astype(np.float32)

        # =====================================================
        # RESAMPLE TO 16 kHz
        # =====================================================

        if self.input_sample_rate != self.target_sample_rate:

            common = gcd(
                self.input_sample_rate,
                self.target_sample_rate
            )

            up = self.target_sample_rate // common
            down = self.input_sample_rate // common

            audio = resample_poly(
                audio,
                up,
                down
            ).astype(np.float32)

        duration = len(audio) / self.target_sample_rate

        self.get_logger().info(
            f'Da ket thuc ghi am: {duration:.2f} giay'
        )

        if duration < 0.5:

            self.get_logger().warning(
                'Ban ghi qua ngan.'
            )

            self.set_processing_done()
            return

        # =====================================================
        # CHECK RAW AUDIO
        # =====================================================

        audio = np.nan_to_num(
            audio,
            nan=0.0,
            posinf=0.0,
            neginf=0.0
        )

        raw_rms = float(
            np.sqrt(np.mean(audio ** 2))
        )

        raw_peak = float(
            np.max(np.abs(audio))
        )

        clipping_ratio = float(
            np.mean(np.abs(audio) >= 0.999)
        )

        self.get_logger().info(
            f'Am thanh goc: RMS={raw_rms:.5f}, '
            f'Peak={raw_peak:.5f}, '
            f'Clipping={clipping_ratio * 100:.3f}%'
        )

        if clipping_ratio > 0.001:

            self.get_logger().warning(
                'Phat hien clipping. '
                'Hay giam gain micro neu am thanh bi meo.'
            )

        # =====================================================
        # SAVE ORIGINAL AUDIO
        # =====================================================

        timestamp = datetime.now().strftime(
            '%Y%m%d_%H%M%S_%f'
        )

        original_path = os.path.join(
            self.save_directory,
            f'stt_{timestamp}_original.wav'
        )

        filtered_path = os.path.join(
            self.save_directory,
            f'stt_{timestamp}_filtered.wav'
        )

        try:

            sf.write(
                original_path,
                audio,
                self.target_sample_rate,
                subtype='PCM_16'
            )

            self.get_logger().info(
                f'Da luu am thanh goc: {original_path}'
            )

        except Exception as e:

            self.get_logger().error(
                f'Loi luu am thanh goc: {e}'
            )

        if raw_rms < self.min_rms:

            self.get_logger().warning(
                'Am thanh qua nho. '
                'Kiem tra micro va Input Volume.'
            )

            self.set_processing_done()
            return

        # =====================================================
        # FILTER AUDIO
        # =====================================================

        try:

            filtered_audio = self.filter_audio(audio)

            filtered_rms = float(
                np.sqrt(np.mean(filtered_audio ** 2))
            )

            filtered_peak = float(
                np.max(np.abs(filtered_audio))
            )

            self.get_logger().info(
                f'Am thanh sau loc: '
                f'RMS={filtered_rms:.5f}, '
                f'Peak={filtered_peak:.5f}'
            )

            sf.write(
                filtered_path,
                filtered_audio,
                self.target_sample_rate,
                subtype='PCM_16'
            )

            self.get_logger().info(
                f'Da luu am thanh sau loc: {filtered_path}'
            )

        except Exception as e:

            self.get_logger().error(
                f'Loc am thanh that bai: {e}. '
                'Su dung am thanh goc.'
            )

            filtered_audio = audio

        # =====================================================
        # TRANSCRIBE IN A WORKER THREAD
        # =====================================================

        worker = threading.Thread(
            target=self.transcribe_audio,
            args=(filtered_audio,),
            daemon=True
        )

        worker.start()

    # =========================================================
    # AUDIO FILTER
    # =========================================================

    def filter_audio(self, audio):

        audio = np.asarray(
            audio,
            dtype=np.float32
        )

        if len(audio) < 1024:
            return audio.copy()

        # 1. Band-pass filter
        # Giam mot phan tieng ù tan so thap va tieng xit cao.
        sos = butter(
            4,
            [80, 7000],
            btype='bandpass',
            fs=self.target_sample_rate,
            output='sos'
        )

        filtered = sosfiltfilt(
            sos,
            audio
        ).astype(np.float32)

        # 2. Giam nhieu nen
        # Khong normalize lon de tranh lam noi bat tieng on.
        filtered = nr.reduce_noise(
            y=filtered,
            sr=self.target_sample_rate,
            stationary=False,
            prop_decrease=self.noise_reduction,
            n_fft=512,
            hop_length=128
        )

        filtered = np.nan_to_num(
            filtered,
            nan=0.0,
            posinf=0.0,
            neginf=0.0
        )

        return filtered.astype(np.float32)

    # =========================================================
    # WHISPER TRANSCRIPTION
    # =========================================================

    def transcribe_audio(self, audio):

        try:

            self.get_logger().info(
                'Dang nhan dang giong noi...'
            )

            segments, info = self.model.transcribe(
                audio,
                language='vi',
                task='transcribe',

                beam_size=5,
                temperature=0.0,

                # Loc cac doan khong co tieng noi
                vad_filter=True,

                vad_parameters={
                    'min_silence_duration_ms': 500,
                    'speech_pad_ms': 250
                },

                # Khong ke thua ngu canh van ban truoc
                condition_on_previous_text=False,

                no_speech_threshold=0.6,

                # Khong dat initial_prompt qua cu the,
                # tranh lam model tu doan sai cau noi.
            )

            recognized_segments = []

            # segments la generator: can duyet het
            for segment in segments:

                text = segment.text.strip()

                if text:

                    recognized_segments.append(text)

                    self.get_logger().info(
                        f'[{segment.start:.2f}s - '
                        f'{segment.end:.2f}s] {text}'
                    )

            recognized_text = ' '.join(
                recognized_segments
            ).strip()

            self.get_logger().info(
                f'Ngon ngu: {info.language}, '
                f'xac suat: {info.language_probability:.3f}'
            )

            if not recognized_text:

                self.get_logger().warning(
                    'Khong nhan dang duoc giong noi.'
                )

                return

            if len(recognized_text) < 3:

                self.get_logger().warning(
                    f'Ket qua qua ngan: {recognized_text}'
                )

                return

            self.get_logger().info(
                f'NHAN DANG: "{recognized_text}"'
            )

            # Gui van ban den AI Bridge
            msg = String()
            msg.data = recognized_text

            self.prompt_publisher.publish(msg)

            self.get_logger().info(
                'Da gui van ban den /ai_prompt.'
            )

        except Exception as e:

            self.get_logger().error(
                f'Loi Faster-Whisper: {e}'
            )

        finally:

            self.set_processing_done()

    # =========================================================
    # PROCESSING STATE
    # =========================================================

    def set_processing_done(self):

        with self.state_lock:
            self.is_processing = False

        self.get_logger().info(
            'STT san sang cho lan ghi am tiep theo.'
        )

    # =========================================================
    # AI RESPONSE
    # =========================================================

    def ai_response_callback(self, msg):

        self.get_logger().info(
            f'AI phan hoi: {msg.data}'
        )

        # Khong tu dong bat micro sau khi AI tra loi.

    # =========================================================
    # CLEANUP
    # =========================================================

    def cleanup(self):

        with self.state_lock:

            self.is_recording = False

            stream = self.audio_stream
            self.audio_stream = None

        if stream is not None:

            try:
                stream.stop()
            except Exception:
                pass

            try:
                stream.close()
            except Exception:
                pass


def main(args=None):

    rclpy.init(args=args)

    node = WhisperSTTNode()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.cleanup()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()