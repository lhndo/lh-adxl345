# Adxl 345 Accelerometer Driver  
Based on **embedded-hal**  
Supports I2C / SPI communications though a generic RegisterBus Trait.  

<br>

Reference: <https://www.analog.com/media/en/technical-documentation/data-sheets/adxl345.pdf>

## Example I2C:
```rust

use lh_adxl345 as adxl;
...
// RP2040 Hal Boilerplate
...
// Setting I2C Pins
let sda_pin: gpio::Pin<_, gpio::FunctionI2c, _> = pins.gpio4.reconfigure();
let scl_pin: gpio::Pin<_, gpio::FunctionI2c, _> = pins.gpio5.reconfigure();

// HAL I2C0 Init
let i2c = hal::I2C::i2c0(
   pac.I2C0,
   sda_pin,
   scl_pin,
   400.kHz(),
   &mut pac.RESETS,
   &sys_clocks.system_clock,
);

// Adxl Bus
let adxlbus = adxl::AdxlBusI2c {
   i2c:  i2c,
   addr: adxl::reg::ADXL_ADDR,
};

// Adxl Device
let mut adxl = adxl::Adxl345::new(adxlbus);

// Adxl Init
if let Err(e) = adxl.init_defaults() {
   println!("Init Err: {}", e);
   return;
}

// Adxl Calibrate
println!("\n Running Calibration ...");
match adxl.calibrate_axis_offsets() {
   Ok((x,y,z)) => {
       println!("Calibration values | X: {x} | Y: {y} | Z: {z} |\n");
   }
   Err(e) => {
       println!("Err: {}", e);
   }
}

// Read Axis in m/s^2
match adxl.read_axis() {
   Ok((x, y, z)) => println!("Axis | X: {x:6.3} | Y: {y:6.3} | Z: {z:6.3} |"),
   Err(e) => println!("Err: {}", e),
}

// Read Axis in raw LSB units
match adxl.read_axis_lsb_units() {
     Ok((x, y, z)) => println!("Axis | X: {x:6} | Y: {y:6} | Z: {z:6} |"),
     Err(e) => println!("Err: {}", e),
 }
 ```


## Example SPI Setup



 ``` rust
 
 use lh_adxl345 as adxl;
 ...

 // RP2040 Hal Boilerplate
 ...

 // ADXL Four wire SPI wiring:
 // SCL >> CLK
 // SDA >> MOSI
 // SDO >> MISO
 // CS  >> GPIO 
 // Note: For power-up in SPI Mode, the ADXL CS Pin
 // has to be pulled to GND with a resistor.

 // SPI Pins
 let spi_mosi = pins.gpio19.into_function::<hal::gpio::FunctionSpi>();
 let spi_miso = pins.gpio16.into_function::<hal::gpio::FunctionSpi>();
 let spi_sclk = pins.gpio18.into_function::<hal::gpio::FunctionSpi>();
 let spi_cs   = pins.gpio17.into_push_pull_output_in_state(gpio::PinState::High);

 // HAL SPI Bus Init
 let spi = hal::spi::Spi::<_, _, _, 8>::new(pac.SPI0, (spi_mosi, spi_miso, spi_sclk));
 let spi = spi.init(
     &mut pac.RESETS,
     clocks.peripheral_clock.freq(),
     2.MHz(),
     embedded_hal::spi::MODE_3,
 );

 // SPI Adxl Bus
 let adxlbus = adxl::AdxlBusSpi {
     spi:    spi,
     spi_cs: spi_cs,
 };

 // Adxl Device
 let mut adxl = adxl::Adxl345::new(adxlbus);

 ...

 ```