function gen_code()
%GEN_CODE Generate portable single-precision FOC C for the STM32F429.
%   Produces clean C source (no MathWorks hardware support package required)
%   that you drop into an STM32CubeMX/HAL project.  Target processor is set
%   to ARM Cortex-M so Embedded Coder emits single-precision math (sinf,
%   cosf, sqrtf, ...) that maps to the F429 FPU.

here = fileparts(mfilename('fullpath'));
addpath(here); cd(here);

cfg = coder.config('lib', 'ecoder', true);
cfg.TargetLang             = 'C';
cfg.GenCodeOnly            = true;
cfg.GenerateReport         = true;
cfg.GenerateExampleMain    = 'DoNotGenerate';
cfg.SupportNonFinite       = false;     % no NaN/Inf checks -> smaller/faster
cfg.PreserveArrayDimensions= true;
cfg.DataTypeReplacement    = 'CoderTypedefs';
cfg.EnableOpenMP           = false;

% ---- target the ARM Cortex-M4F ----
cfg.HardwareImplementation.ProdHWDeviceType = 'ARM Compatible->ARM Cortex-M';
cfg.HardwareImplementation.ProdLongLongMode = true;

% single-precision I/O example types
args = {single(0), single(0), single(0), single(24), single(1500), single(1)};

codegen foc_step_stm32 -config cfg -args args -d codegen_stm32 -report
fprintf('Generated C in %s\n', fullfile(here,'codegen_stm32'));
end
