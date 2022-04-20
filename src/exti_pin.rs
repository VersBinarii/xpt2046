//! Exti trait definition

/// Embedded-hal does not expose any generic interface for
/// working with EXTI interrupts.
/// This trait allows the driver to take control
/// over the EXTI pin and manage the interrupt status.
///
/// Implement it as per following in the app
///
///    pub struct MyIrq<const C: char, const N: u8>(Pin<C, N>);
///
///    impl Xpt2046Exti for MyIrq<'A', 2> {
///        type Exti = EXTI;
///        fn clear_interrupt(&mut self) {
///            self.0.clear_interrupt_pending_bit();
///        }
///
///        fn disable_interrupt(&mut self, exti: &mut Self::Exti) {
///            self.0.disable_interrupt(exti);
///        }
///
///        fn enable_interrupt(&mut self, exti: &mut Self::Exti) {
///            self.0.enable_interrupt(exti);
///        }
///
///        fn is_high(&self) -> bool {
///            self.0.is_high()
///        }
///
///        fn is_low(&self) -> bool {
///            self.0.is_low()
///        }
///    }
///
/// The use it as such in the driver
///
///        let mut xpt_drv = Xpt2046::new(
///            touch_spi,
///            touch_cs,
///            MyIrq(touch_irq),
///            xpt2046::Orientation::PortraitFlipped,
///        );
///
pub trait Xpt2046Exti {
    type Exti;
    fn clear_interrupt(&mut self);
    fn disable_interrupt(&mut self, exti: &mut Self::Exti);
    fn enable_interrupt(&mut self, exti: &mut Self::Exti);
    fn is_high(&self) -> bool;
    fn is_low(&self) -> bool;
}
