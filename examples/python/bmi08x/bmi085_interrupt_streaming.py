#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Copyright (c) 2025 Bosch Sensortec GmbH. All rights reserved.
BSD-3-Clause

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
    contributors may be used to endorse or promote products derived from
    this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
"""
# This is a simple example of BMI085 Interrupt streaming using coinespy
# This example works with Application Board 3.X with BMI085 shuttle board
# pylint: disable=no-member, too-few-public-methods, protected-access, broad-except

from enum import Enum
import coinespy as cpy
import bmi08x_common as bmi08x
from bmi08x_common import BMI08X as BMI08X_CLS
import helper_functions as hfunc


class BMI085(BMI08X_CLS):
    """Child class of BMI08X_CLS with methods for interrupt streaming"""

    def __init__(self, **kwargs):
        super().__init__(kwargs["bus"], kwargs["interface"])
        self.rsp_packet_info_bytes_count = 4
        self.rsp_timestamp_bytes_count = 6

        self.accel_stream_settings = self.get_default_stream_settings(
            0x18, [0x12], [6], 1, cpy.MultiIOPin.SHUTTLE_PIN_8.value, cpy.MultiIOPin.SHUTTLE_PIN_21.value, 1
        )
        self.gyro_stream_settings = self.get_default_stream_settings(
            0x68, [0x02], [6], 2, cpy.MultiIOPin.SHUTTLE_PIN_14.value, cpy.MultiIOPin.SHUTTLE_PIN_22.value, 0
        )
        self.accel_full_range = 16  # ACCEL Range set to 16G
        self.gyro_full_range = 250  # GYRO Range set to 250 dps
        self.number_of_samples = 50
        self.stream_packet_len = {1: 0, 2: 0}

        self.accel_stream_config, self.accel_data_blocks = self.set_stream_settings(
            self.accel_stream_settings, bmi08x.SensorType.ACCEL
        )
        self.gyro_stream_config, self.gyro_data_blocks = self.set_stream_settings(
            self.gyro_stream_settings, bmi08x.SensorType.GYRO
        )
        self.accel_int_config = self.set_accel_interrupt_cfg()
        self.gyro_int_config = self.set_gyro_interrupt_cfg()

    @staticmethod
    def get_default_stream_settings(i2c_addr, reg_x_lsb, no_of_data_bytes, channel_id, cs_pin, int_pin, hw_pin_state):
        """Helper to create default stream settings"""
        return dict(
            I2C_ADDR_PRIMARY=i2c_addr,
            NO_OF_BLOCKS=len(reg_x_lsb),
            REG_X_LSB=reg_x_lsb,
            NO_OF_DATA_BYTES=no_of_data_bytes,
            CHANNEL_ID=channel_id,
            CS_PIN=cs_pin,
            INT_PIN=int_pin,
            INT_TIME_STAMP=1,
            HW_PIN_STATE=hw_pin_state,
        )

    def configure_interrupt(self, int_config, int_channel, int_type, output_mode, level):
        """Helper to configure interrupt settings"""
        int_config["int_channel"] = int_channel
        int_config["int_type"] = int_type
        int_config["int_pin_cfg"] = {
            "output_mode": output_mode,
            "lvl": level,
        }
        return int_config

    def set_accel_interrupt_cfg(self):
        """Set accel interrupt configurations"""
        return self.configure_interrupt(
            dict(),
            bmi08x.Bmi08xAccelIntChannel.BMI08X_INT_CHANNEL_1,
            bmi08x.Bmi08xAccelIntTypes.BMI08X_ACCEL_INT_DATA_RDY,
            bmi08x.BMI08X_INT_MODE_PUSH_PULL,
            bmi08x.BMI08X_INT_ACTIVE_HIGH,
        )

    def set_gyro_interrupt_cfg(self):
        """Set gyro interrupt configurations"""
        return self.configure_interrupt(
            dict(),
            bmi08x.Bmi08xGyroIntChannel.BMI08X_INT_CHANNEL_3,
            bmi08x.Bmi08xGyroIntTypes.BMI08X_GYRO_INT_DATA_RDY,
            bmi08x.BMI08X_INT_MODE_PUSH_PULL,
            bmi08x.BMI08X_INT_ACTIVE_HIGH,
        )

    def set_stream_settings(self, sensor: dict, sensor_type: bmi08x.SensorType):
        """API to configure Stream and Data blocks"""
        stream_config = cpy.StreamingConfig()
        data_blocks = cpy.StreamingBlocks()
        channel_id = sensor["CHANNEL_ID"]
        self.stream_packet_len[channel_id] = 0
        if self.interface == cpy.SensorInterface.I2C:
            stream_config.Intf = cpy.SensorInterface.I2C.value
            stream_config.I2CBus = cpy.I2CBus.BUS_I2C_0.value
            stream_config.DevAddr = sensor["I2C_ADDR_PRIMARY"]

        elif self.interface == cpy.SensorInterface.SPI:
            stream_config.Intf = cpy.SensorInterface.SPI.value
            stream_config.SPIBus = cpy.SPIBus.BUS_SPI_0.value
            stream_config.CSPin = sensor["CS_PIN"]

        if (sensor_type == bmi08x.SensorType.ACCEL and self.interface == cpy.SensorInterface.SPI):
            # extra dummy byte for SPI
            dummy_byte_offset = 1
        else:
            dummy_byte_offset = 0

        data_blocks.NoOfBlocks = sensor["NO_OF_BLOCKS"]
        for i in range(0, data_blocks.NoOfBlocks):
            data_blocks.RegStartAddr[i] = sensor["REG_X_LSB"][i]
            data_blocks.NoOfDataBytes[i] = sensor["NO_OF_DATA_BYTES"][i] + dummy_byte_offset
            self.stream_packet_len[channel_id] = data_blocks.NoOfDataBytes[i]

        stream_config.IntTimeStamp = sensor["INT_TIME_STAMP"]
        stream_config.IntPin = sensor["INT_PIN"]

        self.stream_packet_len[channel_id] += self.rsp_packet_info_bytes_count
        if stream_config.IntTimeStamp:
            self.stream_packet_len[channel_id] += self.rsp_timestamp_bytes_count

        stream_config.HwPinState = sensor["HW_PIN_STATE"]
        # stream_config.SPIType = sensor["SPI_TYPE"]
        # stream_config.ClearOnWrite = sensor["CLEAR_ON_WRITE"]
        # if stream_config.ClearOnWrite:
        #     stream_config.ClearOnWriteConfig.DummyByte = 0
        #     stream_config.ClearOnWriteConfig.StartAddress = 0
        #     stream_config.ClearOnWriteConfig.NumBytesToClear = 0

        # stream_config.IntlineCount = sensor["INTLINE_COUNT"]
        # for i in range(0, stream_config.IntlineCount):
        #     stream_config.IntlineInfo[i] = sensor["INTLINE_INFO"][i]

        return (stream_config, data_blocks)

    def send_stream_settings(self, sensor: dict, sensor_type: Enum):
        """API to send streaming settings to board based on Sensor type"""
        if sensor_type == bmi08x.SensorType.ACCEL:
            ret = self.board.config_streaming(
                sensor["CHANNEL_ID"], self.accel_stream_config, self.accel_data_blocks
            )
        else:
            ret = self.board.config_streaming(
                sensor["CHANNEL_ID"], self.gyro_stream_config, self.gyro_data_blocks
            )
        return ret

    def process_sensor_data(self, data, buffer_index, sensor_type):
        """Helper to process sensor data and convert units"""
        x_data = hfunc.twos_comp((data[1 + buffer_index] << 8) | data[0 + buffer_index], 16)
        y_data = hfunc.twos_comp((data[3 + buffer_index] << 8) | data[2 + buffer_index], 16)
        z_data = hfunc.twos_comp((data[5 + buffer_index] << 8) | data[4 + buffer_index], 16)

        if sensor_type == bmi08x.SensorType.ACCEL:
            return (
                bmi08x.lsb_to_mps2(x_data, self.accel_full_range, 16),
                bmi08x.lsb_to_mps2(y_data, self.accel_full_range, 16),
                bmi08x.lsb_to_mps2(z_data, self.accel_full_range, 16),
            )
        elif sensor_type == bmi08x.SensorType.GYRO:
            return (
                bmi08x.lsb_to_dps(x_data, self.gyro_full_range, 16),
                bmi08x.lsb_to_dps(y_data, self.gyro_full_range, 16),
                bmi08x.lsb_to_dps(z_data, self.gyro_full_range, 16),
            )

    def print_accel_gyro_data(self, sensor: dict, sensor_type, sensor_data):
        """Display Accel and Gyro data after unit conversion"""
        if sensor_data:
            stream_buffer, valid_sample_count = sensor_data
            if stream_buffer:
                if sensor_type == bmi08x.SensorType.ACCEL:
                    print("\nACCEL DATA")
                    print(f"{'Sample_Count':<15}{'Acc_ms2_X':<15}{'Acc_ms2_Y':<15}{'Acc_ms2_Z':<15}{'T(us)':<15}")
                elif sensor_type == bmi08x.SensorType.GYRO:
                    print("\n\nGYRO DATA")
                    print(f"{'Sample_Count':<15}{'Gyr_DPS_X':<15}{'Gyr_DPS_Y':<15}{'Gyr_DPS_Z':<15}{'T(us)':<15}")

                buffer_index = 0
                for _ in range(valid_sample_count):
                    packet_count, buffer_index = hfunc.combine_bytes_to_value(
                        stream_buffer, buffer_index, self.rsp_packet_info_bytes_count
                    )

                    if self.interface == cpy.SensorInterface.SPI and sensor_type == bmi08x.SensorType.ACCEL:
                        buffer_index += 1

                    unit_converted_data = self.process_sensor_data(stream_buffer, buffer_index, sensor_type)
                    buffer_index += 6

                    if sensor["INT_TIME_STAMP"] == 1:
                        time_stamp, buffer_index = hfunc.combine_bytes_to_value(
                            stream_buffer, buffer_index, self.rsp_timestamp_bytes_count
                        )
                        time_stamp //= 30
                    else:
                        time_stamp = 0

                    if sensor_type == bmi08x.SensorType.ACCEL:
                        print(
                            f"{packet_count:<15}{unit_converted_data[0]:<+15.3f}{unit_converted_data[1]:<+15.3f}"
                            f"{unit_converted_data[2]:<+15.3f}{time_stamp:<15}"
                        )
                    elif sensor_type == bmi08x.SensorType.GYRO:
                        print(
                            f"{packet_count:<15}{unit_converted_data[0]:<+15.2f}{unit_converted_data[1]:<+15.2f}"
                            f"{unit_converted_data[2]:<+15.2f}{time_stamp:<15}"
                        )

    def read_stream_data(self, channel_id):
        """Helper to read stream data until the desired number of samples is reached"""
        stream_buffer = []
        total_valid_samples = 0

        while total_valid_samples < self.number_of_samples:
            self.board.error_code, data_buffer, valid_sample_count = self.board.read_stream_sensor_data(
                channel_id, 10
            )
            self.verify_error("Read stream data")

            if valid_sample_count == 0:
                continue

            stream_buffer += data_buffer[:valid_sample_count * self.stream_packet_len[channel_id]]
            total_valid_samples += valid_sample_count

        return stream_buffer, total_valid_samples

    def enable_disable_bmi08x_interrupt(self, enable_interrupt: bool):
        """API to enable interrupt if enable_interrupt is True and disable otherwise

        Args:
            enable_interrupt (bool):
        """
        self.accel_int_config["int_pin_cfg"]["enable_int_pin"] = (
            bmi08x.BMI08X_ENABLE if enable_interrupt else bmi08x.BMI08X_DISABLE
        )
        # Enable accel data ready interrupt channel
        self.api_error_code = bmi08x.bmi08a_set_int_config(self, self.accel_int_config)
        self.verify_api_error("Accel Interrupt Config")

        self.gyro_int_config["int_pin_cfg"]["enable_int_pin"] = (
            bmi08x.BMI08X_ENABLE if enable_interrupt else bmi08x.BMI08X_DISABLE
        )
        # Enable gyro data ready interrupt channel
        self.api_error_code = bmi08x.bmi08g_set_int_config(self, self.gyro_int_config)
        self.verify_api_error("Gyro Interrupt Config")

    def interrupt_streaming(self):
        """API to execute interrupt streaming sequence"""
        # Send streaming settings
        self.board.error_code = self.send_stream_settings(
            self.accel_stream_settings, bmi08x.SensorType.ACCEL
        )
        self.verify_error("Accel Stream settings")
        self.board.error_code = self.send_stream_settings(
            self.gyro_stream_settings, bmi08x.SensorType.GYRO
        )
        self.verify_error("Gyro Stream settings")

        # Wait for 10 ms
        self.board.delay_milli_sec(10)

        # Enable data ready interrupts
        self.enable_disable_bmi08x_interrupt(enable_interrupt=True)

        # Start interrupt streaming
        self.board.error_code = self.board.start_stop_streaming(
            cpy.StreamingMode.STREAMING_MODE_INTERRUPT.value,
            cpy.StreamingState.STREAMING_START.value,
        )
        self.verify_error("Start Interrupt streaming")

        accel_stream_buffer, accel_valid_sample_count = self.read_stream_data(
            self.accel_stream_settings["CHANNEL_ID"]
        )

        gyro_stream_buffer, gyro_valid_sample_count = self.read_stream_data(
            self.gyro_stream_settings["CHANNEL_ID"]
        )

        # Stop interrupt streaming
        self.board.error_code = self.board.start_stop_streaming(
            cpy.StreamingMode.STREAMING_MODE_INTERRUPT.value,
            cpy.StreamingState.STREAMING_STOP.value,
        )
        self.verify_error("Stop Interrupt streaming")

        # Print the streamed accel & gyro data
        if accel_stream_buffer:
            accel_data = [accel_stream_buffer, accel_valid_sample_count]
            self.print_accel_gyro_data(
                self.accel_stream_settings, bmi08x.SensorType.ACCEL, accel_data
            )

        if gyro_stream_buffer:
            gyro_data = [gyro_stream_buffer, gyro_valid_sample_count]
            self.print_accel_gyro_data(
                self.gyro_stream_settings, bmi08x.SensorType.GYRO, gyro_data
            )

        # Flush the buffer
        self.board.flush_interface()

        # Wait for 100 ms
        self.board.delay_milli_sec(100)

        # Disable data ready interrupts
        self.enable_disable_bmi08x_interrupt(enable_interrupt=False)


if __name__ == "__main__":
    # bmi085 = BMI085(bus=cpy.I2CBus.BUS_I2C_0, interface=cpy.SensorInterface.I2C)
    bmi085 = BMI085(bus=cpy.SPIBus.BUS_SPI_0, interface=cpy.SensorInterface.SPI)
    bmi085.init_board()

    # Set Accel power, ODR and range settings
    bmi085.set_accel_power_mode()
    acc_range = bmi085.set_accel_meas_conf()

    # Set Gyro power, ODR and range settings
    bmi085.set_gyro_power_mode()
    gyr_range = bmi085.set_gyro_meas_conf()
    print(f"Accel range: {acc_range} Gyro range: {gyr_range}")

    if (
        bmi085.accel_cfg["POWER_MODE"] == bmi08x.BMI08X_ACCEL_PM_SUSPEND
        and (
            bmi085.gyro_cfg["POWER_MODE"] == bmi08x.BMI08X_GYRO_PM_SUSPEND
            or bmi085.gyro_cfg["POWER_MODE"] == bmi08x.BMI08X_GYRO_PM_DEEP_SUSPEND
        )
    ):
        print(
            "Warning: Accel and gyro sensors are in suspend mode. Use them in active/normal mode !!"
        )

    try:
        bmi085.interrupt_streaming()
    except Exception as e:
        print(f"Exception:{e}")
        bmi085.close_comm()
    bmi085.close_comm()
