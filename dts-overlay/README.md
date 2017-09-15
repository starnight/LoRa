# SX1278 Compatible Radio Device Tree

## Required properties:
  - compatible:		should be "semtech,sx1276", "semtech,sx1277",
			"semtech,sx1277" or "semtech,sx1279" depends on your
			transceiver board
  - spi-max-frequency:	maximal bus speed, should be set something under or
			equal 10000000 Hz
  - reg:		the chipselect index
  - clock-frequency:	the external crystal oscillator frequency in Hz of the
			transceiver

## Optional properties:
  - center-carrier-frq:	the RF center carrier frequency in Hz
  - rf-bandwidth:	the RF bandwidth in Hz
  - minimal-RF-channel:	the minimal RF channel number and the value must be with
			prefix "/bits/ 8" because of being a byte datatype
  - maximum-RF-channel: the maximum RF channel number and the value must be with
			prefix "/bits/ 8" because of being a byte datatype
  - spreading-factor:	the spreading factor of Chirp Spread Spectrum modulation

## Example:

	sx1278@0 {
		compatible = "semtech,sx1278";
		spi-max-frequency = <15200>;
		reg = <0>;
		clock-frequency = <32000000>;
		minimal-RF-channel = /bits/ 8 <11>;
		maximum-RF-channel = /bits/ 8 <11>;
	};
