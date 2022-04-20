#![no_main]
#![no_std]

#[rtic::app(device = stm32f4xx_hal::pac)]
mod app {
    use stm32f4xx_hal::{
        gpio::{Alternate, Edge, Input, Output, Pin, PushPull},
        pac::{EXTI, SPI1, TIM1},
        prelude::*,
        spi::{Mode, Phase, Polarity, Spi},
        timer::Delay,
    };
    use xpt2046::{self, Xpt2046, Xpt2046Exti};

    pub struct MyIrq<const C: char, const N: u8>(Pin<C, N, Input>);

    impl Xpt2046Exti for MyIrq<'A', 2> {
        type Exti = EXTI;
        fn clear_interrupt(&mut self) {
            self.0.clear_interrupt_pending_bit();
        }

        fn disable_interrupt(&mut self, exti: &mut Self::Exti) {
            self.0.disable_interrupt(exti);
        }

        fn enable_interrupt(&mut self, exti: &mut Self::Exti) {
            self.0.enable_interrupt(exti);
        }

        fn is_high(&self) -> bool {
            self.0.is_high()
        }

        fn is_low(&self) -> bool {
            self.0.is_low()
        }
    }
    type TouchSpi = Spi<
        SPI1,
        (
            Pin<'A', 5, Alternate<5>>,
            Pin<'A', 6, Alternate<5>>,
            Pin<'A', 7, Alternate<5>>,
        ),
    >;
    #[shared]
    struct Shared {
        xpt_drv: Xpt2046<TouchSpi, Pin<'A', 4, Output<PushPull>>, MyIrq<'A', 2>>,
        exti: EXTI,
    }

    #[local]
    struct Local {
        delay: Delay<TIM1, 1000000>,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut dp = ctx.device;
        let cp = ctx.core;

        let rcc = dp.RCC.constrain();
        let clocks = rcc.cfgr.use_hse(25.MHz()).sysclk(100.MHz()).freeze();

        let gpioa = dp.GPIOA.split();
        let gpiob = dp.GPIOB.split();
        let gpioc = dp.GPIOC.split();

        let mode = Mode {
            polarity: Polarity::IdleLow,
            phase: Phase::CaptureOnFirstTransition,
        };
        let mut delay = dp.TIM1.delay_us(&clocks);

        // Touch interface
        let mut touch_irq = gpioa.pa2.into_pull_up_input();
        let mut syscfg = dp.SYSCFG.constrain();
        touch_irq.make_interrupt_source(&mut syscfg);
        touch_irq.trigger_on_edge(&mut dp.EXTI, Edge::Falling);
        touch_irq.enable_interrupt(&mut dp.EXTI);
        let touch_cs = gpioa.pa4.into_push_pull_output();
        let touch_clk = gpioa.pa5.into_alternate();
        let touch_mosi = gpioa.pa7.into_alternate().internal_pull_up(true);
        let touch_miso = gpioa.pa6.into_alternate();
        let touch_spi = Spi::new(
            dp.SPI1,
            (touch_clk, touch_miso, touch_mosi),
            mode,
            2.MHz(),
            &clocks,
        );

        let mut xpt_drv = Xpt2046::new(
            touch_spi,
            touch_cs,
            MyIrq(touch_irq),
            xpt2046::Orientation::PortraitFlipped,
        );
        xpt_drv.init(&mut delay);

        (
            Shared {
                xpt_drv,
                exti: dp.EXTI,
            },
            Local { delay },
            init::Monotonics(),
        )
    }

    #[idle(local = [delay], shared = [xpt_drv, exti])]
    fn idle(ctx: idle::Context) -> ! {
        let mut xpt_drv = ctx.shared.xpt_drv;
        let mut exti = ctx.shared.exti;
        let mut delay = ctx.local.delay;

        loop {
            xpt_drv.lock(|xpt| {
                exti.lock(|e| xpt.run(e));
                if xpt.is_touched() {
                    let p = xpt.get_touch_point();
                    defmt::println!("x:{} y:{}", p.x, p.y);
                }
            });
            delay.delay_ms(1u32);
        }
    }

    #[task(binds = EXTI2, local = [], shared = [xpt_drv, exti])]
    fn exti2(ctx: exti2::Context) {
        let xpt = ctx.shared.xpt_drv;
        let exti = ctx.shared.exti;
        (xpt, exti).lock(|xpt, exti| xpt.exti_irq_handle(exti))
    }
}
