# ct-stepgen-dev
2.476.101.01 Curve Tracer Step Generator Analog Frontend Firmware on STM32f746

---

- `Project > Options for Target "xx" --> Tab [ C/C++ ]` 
    + `Preprocessor Symbols --> Define: ARM_MATH_CM7`
    + `Include Paths --> "../Drivers/CMSIS/DSP"`
- `Project > Manage > Run-Time Environment`

```
- CMSIS
--- DSP [check] [library]
```