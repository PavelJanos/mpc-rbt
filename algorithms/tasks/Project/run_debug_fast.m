function run_debug_fast(varargin)
%RUN_DEBUG_FAST Run project_tuning_debug with warm-session defaults.
%   RUN_DEBUG_FAST() uses fast defaults for iterative debugging.
%   RUN_DEBUG_FAST('Map','indoor_1','View',true,'Timeout',2050) overrides
%   selected environment-backed settings before calling project_tuning_debug.

cfg = struct( ...
    'Map', 'indoor_1', ...
    'Repeat', 1, ...
    'View', true, ...
    'Controller', 'pure_pursuit', ...
    'Smoothing', 'iterative', ...
    'Localization', 'fusion', ...
    'Timeout', 2050, ...
    'Warm', true, ...
    'SaveOutputs', true);

if mod(nargin, 2) ~= 0
    error('run_debug_fast:InvalidArgs', 'Use name/value pairs.');
end

for idx = 1:2:nargin
    name = varargin{idx};
    value = varargin{idx + 1};
    if ~ischar(name) && ~isstring(name)
        error('run_debug_fast:InvalidName', 'Argument names must be text.');
    end
    key = char(name);
    if ~isfield(cfg, key)
        error('run_debug_fast:UnknownOption', 'Unknown option: %s', key);
    end
    cfg.(key) = value;
end

setenv('PROJECT_DEBUG_MAP', char(string(cfg.Map)));
setenv('PROJECT_DEBUG_REPEAT', sprintf('%d', round(cfg.Repeat)));
setenv('PROJECT_DEBUG_VIEW', logical_to_env(cfg.View));
setenv('PROJECT_CONTROLLER_MODE', char(string(cfg.Controller)));
setenv('PROJECT_SMOOTHING_MODE', char(string(cfg.Smoothing)));
setenv('PROJECT_LOCALIZATION_MODE', char(string(cfg.Localization)));
setenv('PROJECT_DEBUG_TIMEOUT', sprintf('%d', round(cfg.Timeout)));
setenv('PROJECT_DEBUG_WARM', logical_to_env(cfg.Warm));
setenv('PROJECT_DEBUG_SAVE_OUTPUTS', logical_to_env(cfg.SaveOutputs));

project_dir = fileparts(mfilename('fullpath'));
if ~strcmpi(pwd, project_dir)
    cd(project_dir);
end

project_tuning_debug;

end

function env_value = logical_to_env(value)
if islogical(value)
    env_value = ternary(value, '1', '0');
elseif isnumeric(value)
    env_value = ternary(value ~= 0, '1', '0');
else
    text = lower(strtrim(char(string(value))));
    is_true = any(strcmp(text, {'1', 'true', 'on', 'yes'}));
    env_value = ternary(is_true, '1', '0');
end
end

function out = ternary(condition, true_value, false_value)
if condition
    out = true_value;
else
    out = false_value;
end
end
