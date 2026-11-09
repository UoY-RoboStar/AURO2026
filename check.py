#!/usr/bin/env python3
#
# Copyright (c) 2025-2026 University of York and others
#
# Command line utility supporting the AURO2026 assessment: checks a workspace before it is
# submitted, runs its scenarios, installs its dependencies, and builds, cleans and zips it.
#
# One devcontainer.json (JSON with comments) of the workspace, .devcontainer/devcontainer.json or
# .devcontainer/<name>/devcontainer.json, lists, under "customizations": {"auro": {...}}:
#   "apt": [...], "pip": [...]   packages the solution needs, for install-dependencies;
#   "scenarios": [{"name": ..., "description": ..., "parameters": {"task": "task2", ...}}, ...]
#                                launch arguments of solution.launch.py to run it with; without
#                                a task, a scenario is of task2.
#
#   ./check.py check-submission     run every check below, on this workspace
#   ./check.py list-scenarios [--task task2]   the scenarios in .devcontainer/devcontainer.json
#   ./check.py run-scenario N       launch the solution with scenario N (or its name)
#   ./check.py install-dependencies install the apt and pip packages devcontainer.json lists
#   ./check.py list-launch          launch files and the arguments they declare
#   ./check.py build | clean        colcon build, or remove build/, install/ and log/
#   ./check.py zip-workspace OUT    zip the workspace, without build/, install/, log/ and .git/
#   ./check.py check-zip ZIP        whether a ZIP holds a ROS 2 workspace
#
# check-submission checks that:
#   * solution/launch/solution.launch.py supports (declares) every launch argument of
#     assessment/launch/assessment.launch.py, so that the solution can be run with any of them.
#     It need not set them: an empty value means not set, and the assessment's default applies;
#   * only one devcontainer.json has "auro" customizations, and exactly one of its scenarios is
#     of task1;
#   * the launch arguments of the solution's runs are valid, as assessment.launch.py would check
#     them: each task with solution.launch.py's defaults (in task1, e.g., nav2, nav2_start and
#     keepout_mask have to make sense together, and a map has to exist), and every scenario;
#   * nothing in the solution's packages (solution, and the packages of the workspace it depends
#     on, e.g. solution_interfaces) uses Gazebo's transport (gz-transport), directly or
#     through a bridge or the gz command line: the solution has to use the ROS interfaces only, in
#     both tasks;
#   * the packages of the assessment are as they are upstream.
# Run it with the workspace built and sourced (source install/setup.bash).
#
# Contributors:
#   * Pedro Ribeiro - initial implementation (rcutil.py, AURO2025)

import argparse
import json
import os
from pathlib import Path
import re
import shlex
import shutil
import subprocess
import sys
import zipfile

try:
    import argcomplete
except ImportError:
    argcomplete = None

VERSION = '2.0'

UPSTREAM_URL = 'https://github.com/UoY-RoboStar/AURO2026'
UPSTREAM_BRANCH = 'main'

# The packages that come with the assessment: they have to be as they are upstream, and are not
# part of the solution
ASSESSMENT_PACKAGES = ['assessment', 'assessment_interfaces', 'auro_gz_plugins']
# The solution: this package, and the packages of the workspace it depends on
SOLUTION_PACKAGE = 'solution'
THIS_TOOL = 'check.py'

ASSESSMENT_LAUNCH = Path('assessment/launch/assessment.launch.py')
SOLUTION_LAUNCH = Path('solution/launch/solution.launch.py')
# Arguments of assessment.launch.py that solution.launch.py need not support: use_sim_time is
# always true, whatever the value given
IGNORED_ARGUMENTS = {'use_sim_time'}

DEVCONTAINER = Path('.devcontainer/devcontainer.json')
TASKS = ('task1', 'task2')
DEFAULT_SCENARIO_TASK = 'task2'  # of a scenario without a task parameter

BUILD_DIRS = {'build', 'install', 'log'}
SKIPPED_DIRS = BUILD_DIRS | {'.git', '__pycache__'}

# Uses of Gazebo's transport, in any file of the solution's packages: (what it is, pattern)
GAZEBO_PATTERNS = [
    ("Gazebo's Python bindings (gz.transport, gz.msgs)",
     re.compile(r'^\s*(?:from\s+gz(?:\.\w+)*\s+import\b|import\s+gz\b)')),
    ("Gazebo's Python bindings, imported dynamically",
     re.compile(r'''(?:import_module|__import__)\(\s*['"]gz\b''')),
    ('the gz-transport library', re.compile(r'\b(?:gz|ignition)(?:/|::|[-_])transport')),
    ('the gz-msgs library', re.compile(r'\b(?:gz|ignition)(?:/|::|[-_])msgs')),
    ('a ROS-Gazebo bridge', re.compile(r'\bros_gz_(?:bridge|sim|image|point_cloud)\b')),
    ('a ROS-Gazebo bridge configuration',
     re.compile(r'\bgz_(?:topic|type|req_type|rep_type)_name\b')),
    ("Gazebo's command line",
     re.compile(r'''\b(?:gz|ign)\s+(?:service|topic|model|sim|log)\b'''
                r'''|['"](?:gz|ign)['"]\s*,\s*['"](?:service|topic|model|sim|log)\b''')),
]
# Lines that are comments, by file type: in C and C++, # starts preprocessor lines (#include), not
# comments
C_SUFFIXES = {'.c', '.cc', '.cpp', '.cxx', '.h', '.hh', '.hpp', '.hxx'}
XML_SUFFIXES = {'.xml', '.sdf', '.urdf', '.xacro', '.world', '.launch'}
C_COMMENTS, XML_COMMENTS, HASH_COMMENTS = ('//', '/*', '*'), ('<!--',), ('#',)
SCANNED_SIZE_LIMIT = 1024 * 1024  # bytes: larger files are data, not code
UNSCANNED_SUFFIXES = {'.md', '.rst', '.txt', '.pgm', '.png', '.jpg', '.dae', '.stl'}

# ANSI colours, on a terminal only
if sys.stdout.isatty():
    RED, GREEN, YELLOW, BLUE, RESET = '\033[91m', '\033[92m', '\033[93m', '\033[94m', '\033[0m'
else:
    RED = GREEN = YELLOW = BLUE = RESET = ''


def ok(message):
    print(f'{GREEN}✅ {message}{RESET}')


def warn(message):
    print(f'{YELLOW}⚠️  {message}{RESET}')


def fail(message):
    print(f'{RED}❌ {message}{RESET}')


# -----------------------------
# Workspace
# -----------------------------
def walk(path):
    """os.walk(path), without build, install, log, .git and __pycache__ directories."""
    for root, dirs, files in os.walk(path):
        dirs[:] = [d for d in dirs if d not in SKIPPED_DIRS]
        yield root, dirs, files


def packages(workspace_path: Path):
    """Return {package name: its directory} of every package in a workspace."""
    found = {}
    for root, dirs, files in walk(workspace_path):
        if 'package.xml' in files:
            text = (Path(root) / 'package.xml').read_text(errors='replace')
            match = re.search(r'<name>\s*([^<\s]+)\s*</name>', text)
            found[match.group(1) if match else Path(root).name] = Path(root)
            dirs[:] = []  # packages do not nest
    return found


def is_ros2_workspace(path: Path) -> bool:
    return path.is_dir() and bool(packages(path))


def is_ros2_workspace_zip(zip_path: Path) -> bool:
    """Whether a ZIP file holds a ROS 2 workspace: a package.xml anywhere in it."""
    with zipfile.ZipFile(zip_path, 'r') as z:
        return any(Path(name).name == 'package.xml' for name in z.namelist()
                   if not name.endswith('/'))


def workspace_or_exit(path):
    workspace_path = Path(path).resolve()
    if not is_ros2_workspace(workspace_path):
        fail(f'Not a ROS 2 workspace: {workspace_path}')
        sys.exit(1)
    return workspace_path


# -----------------------------
# Colcon build
# -----------------------------
def colcon_build(workspace_path: Path, colcon_args=None):
    colcon_args = colcon_args or ['--symlink-install']
    cmd_list = ['colcon', 'build'] + colcon_args
    ros2_setup = f"/opt/ros/{os.environ.get('ROS_DISTRO', 'lyrical')}/setup.bash"
    print(f"Running: {' '.join(cmd_list)} in {workspace_path}")

    # Built against ROS alone, not against whatever workspace is sourced
    env = os.environ.copy()
    for var in ['AMENT_PREFIX_PATH', 'CMAKE_PREFIX_PATH']:
        env.pop(var, None)
    cmd_str = f'source {ros2_setup} && ' + ' '.join(shlex.quote(arg) for arg in cmd_list)
    subprocess.run(cmd_str, cwd=workspace_path, shell=True, env=env, executable='/bin/bash',
                   check=True)


def clean_ros2_workspace(workspace_path: Path):
    """Delete a workspace's build/, install/ and log/ directories."""
    print(f'Cleaning ROS 2 workspace at: {workspace_path}\n')
    for d in sorted(BUILD_DIRS):
        path = workspace_path / d
        if path.exists():
            print(f'Deleting {path} ...')
            shutil.rmtree(path)
        else:
            print(f'{path} does not exist, skipping.')
    print('\nOpen a new terminal before building again: this one still has the old workspace '
          'sourced.')


# -----------------------------
# Launch files
# -----------------------------
# Run in a separate process, with the workspace sourced: loads launch files, and the assessment's
# own checks of launch argument values. Reads a JSON request on stdin, writes a JSON reply.
INSPECT_CODE = r"""
import contextlib, importlib.util, io, json, os, sys
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.utilities import perform_substitutions

request = json.load(sys.stdin)
reply = {}
# Arguments of assessment.launch.py that only one task uses: a run of the other task gets the
# assessment's defaults for them, as solution.launch.py passes on only the task's own, unless
# they are given explicitly
TASK_ONLY = {
    'task1': {'nav2', 'nav2_start', 'nav2_params', 'keepout_mask', 'end_effectors'},
    'task2': {'target_power_level', 'maximum_power_level', 'control_rod_coefficient',
              'power_display', 'reactor_fill_percentage', 'replacement_percentage',
              'initial_mean_level', 'initial_level_distribution', 'initial_level_spread',
              'replacement_level', 'winch_camera_rate', 'winch_camera_noise',
              'winch_failure_probability', 'winch_cable'},
}

def load(path):
    spec = importlib.util.spec_from_file_location(
        'launch_module_' + str(abs(hash(path))), path)
    module = importlib.util.module_from_spec(spec)
    with contextlib.redirect_stdout(io.StringIO()):
        spec.loader.exec_module(module)
        description = module.generate_launch_description()
    arguments = {}
    for entity in description.entities:
        if isinstance(entity, DeclareLaunchArgument):
            default = entity.default_value
            arguments[entity.name] = {
                'default': None if default is None
                else perform_substitutions(LaunchContext(), default),
                'choices': entity.choices}
    return module, arguments

def check(module, assessment, solution, overrides):
    # Errors in the values a run would give assessment.launch.py
    from ament_index_python.packages import get_package_share_directory
    from assessment.tasks import (existing_file, load_simulation_config, parse_keepout_mask,
                                  parse_nav2, parse_nav2_params, parse_nav2_start,
                                  parse_time_limit, TASKS)
    errors = []
    unknown = sorted(set(overrides) - set(solution))
    if unknown:
        errors.append(f'solution.launch.py has no arguments {", ".join(unknown)}')
    task = overrides.get('task', solution.get('task', {}).get('default'))
    if task not in TASKS:
        return errors + [f'task must be one of {list(TASKS)}, got {task!r}']
    other = set().union(*(names for t, names in TASK_ONLY.items() if t != task))
    # An empty value is not set: solution.launch.py does not pass it on, so the assessment's
    # default applies
    values = {name: a['default'] for name, a in assessment.items()}
    values.update({name: a['default'] for name, a in solution.items()
                   if name not in other and a['default'] != ''})
    values.update({name: value for name, value in overrides.items() if value != ''})
    for name, a in assessment.items():
        if name in other and name not in overrides:
            continue
        if a['choices'] and values.get(name) not in a['choices']:
            errors.append(f'{name} must be one of {a["choices"]}, got {values.get(name)!r}')

    def attempt(function, *args):
        try:
            return function(*args)
        except (RuntimeError, ValueError) as error:
            errors.append(str(error))

    # Nav2 is for task1: in task2, only what a scenario sets explicitly counts
    nav2 = ('nav2', 'nav2_start', 'keepout_mask', 'nav2_params')
    if task == 'task1' or any(name in overrides for name in nav2):
        mode = (attempt(parse_nav2, task, values['nav2']) or (None, None))[0]
        attempt(parse_nav2_start, mode, values['nav2_start'])
        attempt(parse_keepout_mask, mode, values['keepout_mask'])
        attempt(parse_nav2_params, values['nav2_params'])
    if values.get('rviz_config_file'):
        attempt(existing_file, 'rviz_config_file', values['rviz_config_file'])
    if 'time_limit' in values:
        attempt(parse_time_limit, values['time_limit'])
    for name in ('target_power_level', 'control_rod_coefficient', 'maximum_power_level'):
        value = values.get(name)
        if name == 'maximum_power_level' and value == '':
            continue
        try:
            float(value)
        except (TypeError, ValueError):
            errors.append(f'{name} must be a number, got {value!r}')
    # The world as the launch would make it: checks the numbers of the fuel rods, winch and
    # random seed, and the simulation settings
    share = get_package_share_directory('assessment')
    try:
        config = load_simulation_config(os.path.join(share, 'config', 'simulation.yaml'))
        with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            module.prepare_world(
                os.path.join(share, 'worlds', 'assessment.world'),
                {name: values[name] for name in module.WORLD_ARGS}, task, config,
                values.get('end_effectors', 'true') == 'true',
                values.get('limit_real_time_factor', 'true') == 'true')
    except (RuntimeError, ValueError, KeyError) as error:
        errors.append(str(error))
    return errors

try:
    if 'arguments_of' in request:
        reply['arguments'] = load(request['arguments_of'])[1]
    else:
        module, assessment = load(request['assessment'])
        _, solution = load(request['solution'])
        reply['assessment'], reply['solution'] = assessment, solution
        reply['errors'] = {label: check(module, assessment, solution, overrides)
                           for label, overrides in request['cases'].items()}
except Exception as error:  # reported by the caller
    reply = {'failure': f'{type(error).__name__}: {error}'}
print('INSPECT ' + json.dumps(reply), flush=True)
os._exit(0)  # without waiting for threads the launch files started
"""


def inspect(request, timeout=120):
    """
    Run INSPECT_CODE on a request, in a separate process, and return its reply.

    Separate, as launch files change the environment. Raises RuntimeError if it fails.
    """
    try:
        # -P: not from the current directory, where the workspace's source folders would hide
        # the installed packages of the same names
        result = subprocess.run([sys.executable, '-P', '-c', INSPECT_CODE],
                                input=json.dumps(request), capture_output=True, text=True,
                                timeout=timeout)
    except subprocess.TimeoutExpired:
        raise RuntimeError('loading the launch files took too long')
    for line in result.stdout.splitlines():
        if line.startswith('INSPECT '):
            reply = json.loads(line[len('INSPECT '):])
            if 'failure' in reply:
                raise RuntimeError(f"{reply['failure']} (is the workspace built and sourced?)")
            return reply
    error = result.stderr.strip().splitlines()
    raise RuntimeError(f"{error[-1] if error else 'no output'} (is the workspace built and "
                       'sourced?)')


def launch_arguments(launch_path: Path):
    """Return {name: {'default', 'choices'}} of the arguments a Python launch file declares."""
    if not launch_path.is_file():
        raise RuntimeError(f'no such file: {launch_path}')
    return inspect({'arguments_of': str(launch_path)})['arguments']


def check_argument_values(workspace_path: Path, cases):
    """
    Return {label: errors} of runs of the solution, each {label: launch arguments given}.

    The values a run gives assessment.launch.py are its defaults, then solution.launch.py's, then
    those given; they are checked as the launch would check them.
    """
    return inspect({'assessment': str(workspace_path / ASSESSMENT_LAUNCH),
                    'solution': str(workspace_path / SOLUTION_LAUNCH),
                    'cases': cases})['errors']


def list_launch_arguments(workspace_path: Path):
    """Return {launch file: its arguments, or the error loading it} of a workspace."""
    launch_info = {}
    for root, _, files in walk(workspace_path):
        if 'launch' not in Path(root).relative_to(workspace_path).parts:
            continue
        for f in sorted(files):
            if f.endswith('.py'):
                path = Path(root) / f
                try:
                    launch_info[str(path.relative_to(workspace_path))] = list(
                        launch_arguments(path))
                except RuntimeError as error:
                    launch_info[str(path.relative_to(workspace_path))] = f'⚠️  {error}'
    return launch_info


def check_launch_arguments(workspace_path: Path):
    """Check that solution.launch.py has every argument of assessment.launch.py."""
    try:
        required = launch_arguments(workspace_path / ASSESSMENT_LAUNCH)
        supported = launch_arguments(workspace_path / SOLUTION_LAUNCH)
    except RuntimeError as error:
        fail(f'Launch arguments: {error}')
        return False
    missing = [name for name in required
               if name not in supported and name not in IGNORED_ARGUMENTS]
    if missing:
        fail(f'Launch arguments: {SOLUTION_LAUNCH} does not support these arguments of '
             f'{ASSESSMENT_LAUNCH.name}: {", ".join(missing)}. Add them to its '
             "ASSESSMENT_ARGUMENTS, with an empty value to leave them at the assessment's "
             'defaults (ros2 launch assessment assessment.launch.py --show-args).')
        return False
    needed = [name for name in required if name not in IGNORED_ARGUMENTS]
    ok(f'Launch arguments: {SOLUTION_LAUNCH} supports all {len(needed)} arguments of '
       f'{ASSESSMENT_LAUNCH.name}.')
    return True


# -----------------------------
# Gazebo transport
# -----------------------------
def gazebo_uses(package_path: Path):
    """Yield (file, line number, what, line) of every use of Gazebo's transport in a package."""
    for root, _, files in walk(package_path):
        for f in sorted(files):
            path = Path(root) / f
            if path.suffix.lower() in UNSCANNED_SUFFIXES and f != 'CMakeLists.txt':
                continue
            try:
                if path.stat().st_size > SCANNED_SIZE_LIMIT:
                    continue
                text = path.read_text(encoding='utf-8')
            except (OSError, UnicodeDecodeError):
                continue  # not text
            suffix = path.suffix.lower()
            comments = (C_COMMENTS if suffix in C_SUFFIXES
                        else XML_COMMENTS if suffix in XML_SUFFIXES else HASH_COMMENTS)
            for number, line in enumerate(text.splitlines(), 1):
                if line.strip().startswith(comments):
                    continue
                for what, pattern in GAZEBO_PATTERNS:
                    if pattern.search(line):
                        yield path, number, what, line.strip()
                        break


def solution_packages(workspace_path: Path):
    """
    Return {name: directory} of the solution's packages.

    That is the solution package, and every package of the workspace it depends on, directly or
    not, other than the assessment's.
    """
    found = packages(workspace_path)
    pending, solution = [SOLUTION_PACKAGE], {}
    while pending:
        name = pending.pop()
        if name in solution or name not in found or name in ASSESSMENT_PACKAGES:
            continue
        solution[name] = found[name]
        text = (found[name] / 'package.xml').read_text(errors='replace')
        pending += re.findall(r'<(?:\w+_)?depend>\s*([^<\s]+)\s*</', text)
    return solution


def check_gazebo_transport(workspace_path: Path):
    """Check that no package of the solution's uses Gazebo's transport."""
    solution_packages_ = solution_packages(workspace_path)
    if SOLUTION_PACKAGE not in solution_packages_:
        fail(f'Gazebo transport: there is no package {SOLUTION_PACKAGE!r} in the workspace.')
        return False
    uses = [use for path in solution_packages_.values() for use in gazebo_uses(path)]
    if uses:
        fail("Gazebo transport: the solution uses Gazebo's transport, which it must not, in "
             'either task: use the ROS topics and services of the assessment instead.')
        for path, number, what, line in uses:
            print(f'     {path.relative_to(workspace_path)}:{number}: {what}: {line[:100]}')
        return False
    ok(f"Gazebo transport: not used by the solution's packages "
       f"({', '.join(sorted(solution_packages_))}).")
    return True


# -----------------------------
# Changes to the assessment's packages
# -----------------------------
def git(workspace_path, *args, check=False):
    return subprocess.run(['git', *args], cwd=workspace_path, capture_output=True, text=True,
                          check=check)


def changed_files(workspace_path: Path, url: str, branch: str, folders):
    """
    Return {folder: files in it that differ from the upstream branch}, untracked ones included.

    This compares the files as they are, committed or not, with the branch. A difference is
    either a change made here or a change upstream that is not here yet. Raises RuntimeError if
    the branch cannot be fetched.
    """
    temp_ref = f'refs/remotes/check_upstream/{branch}'
    fetch = git(workspace_path, 'fetch', '--depth=1', '--no-tags', url, f'+{branch}:{temp_ref}')
    if fetch.returncode != 0:
        raise RuntimeError(f'could not fetch {branch} of {url}: '
                           f"{(fetch.stderr.strip().splitlines() or ['?'])[-1]}")
    try:
        changes = {}
        for folder in folders:
            diff = git(workspace_path, 'diff', '--name-only', temp_ref, '--', folder)
            untracked = git(workspace_path, 'ls-files', '--others', '--exclude-standard', '--',
                            folder)
            changes[folder] = sorted(set(diff.stdout.split()) | set(untracked.stdout.split()))
        return changes
    finally:
        git(workspace_path, 'update-ref', '-d', temp_ref)


def check_assessment_packages(workspace_path: Path, url: str, branch: str):
    """Check that the assessment's packages are as they are upstream; mention a newer tool."""
    if git(workspace_path, 'rev-parse', '--is-inside-work-tree').returncode != 0:
        warn(f'Assessment packages: not checked, as {workspace_path} is not a Git repository.')
        return True
    try:
        changes = changed_files(workspace_path, url, branch, ASSESSMENT_PACKAGES + [THIS_TOOL])
    except RuntimeError as error:
        warn(f'Assessment packages: not checked: {error}')
        return True
    if changes.pop(THIS_TOOL):
        warn(f'{THIS_TOOL} differs from the one in {url}: there may be a newer version.')
    changed = {folder: files for folder, files in changes.items() if files}
    if changed:
        fail(f'Assessment packages: these differ from {branch} of {url}. Undo your changes '
             '(git checkout or git restore), or bring in the newer upstream version (git pull):')
        for folder, files in changed.items():
            print(f'     {folder}: {len(files)} files, e.g. {", ".join(files[:3])}')
        return False
    ok(f"Assessment packages: {', '.join(ASSESSMENT_PACKAGES)} are as in {branch} of {url}.")
    return True


# -----------------------------
# Launch argument values
# -----------------------------
def report_argument_errors(errors_by_case):
    """Print the errors of each case; return whether there were none."""
    for label, errors in errors_by_case.items():
        if errors:
            fail(f'Launch argument values: {label}:')
            for error in errors:
                print(f'     {error}')
        else:
            ok(f'Launch argument values: {label}: valid.')
    return not any(errors_by_case.values())


def check_solution_defaults(workspace_path: Path, scenarios):
    """
    Check the launch argument values of the solution's runs.

    Those are each task with solution.launch.py's defaults (task1, e.g., with its nav2 and
    nav2_start, as they are), and every scenario in devcontainer.json.
    """
    cases = {f'{task} with the defaults of {SOLUTION_LAUNCH.name}': {'task': task}
             for task in TASKS}
    cases.update({f'scenario {i}, {scenario_name(s, i)}': scenario_parameters(s)
                  for i, s in enumerate(scenarios, 1)})
    try:
        return report_argument_errors(check_argument_values(workspace_path, cases))
    except RuntimeError as error:
        fail(f'Launch argument values: {error}')
        return False


# -----------------------------
# devcontainer.json: dependencies and scenarios
# -----------------------------
def strip_jsonc(text: str) -> str:
    """Return JSON with comments (//, /* */) and trailing commas, as plain JSON."""
    out, i, n = [], 0, len(text)
    while i < n:
        c = text[i]
        if c == '"':  # a string, copied as it is, // and all
            j = i + 1
            while j < n and text[j] != '"':
                j += 2 if text[j] == '\\' else 1
            out.append(text[i:j + 1])
            i = j + 1
        elif text.startswith('//', i):
            i = text.find('\n', i)
            i = n if i < 0 else i
        elif text.startswith('/*', i):
            i = text.find('*/', i + 2)
            i = n if i < 0 else i + 2
        else:
            out.append(c)
            i += 1
    return re.sub(r',(\s*[}\]])', r'\1', ''.join(out))


def read_jsonc(path: Path):
    """Return the contents of a JSON file with comments; raise RuntimeError if it cannot."""
    try:
        return json.loads(strip_jsonc(path.read_text(encoding='utf-8')))
    except OSError as error:
        raise RuntimeError(f'cannot read {path}: {error.strerror}')
    except json.JSONDecodeError as error:
        raise RuntimeError(f'{path} is not valid JSON: {error.msg} (line {error.lineno}, '
                           f'column {error.colno})')


def devcontainers_with_auro(workspace_path: Path):
    """
    Return the devcontainer.json files of the workspace that have "auro" customizations.

    Those are .devcontainer/devcontainer.json and .devcontainer/<name>/devcontainer.json, the
    configurations VS Code offers. Raises RuntimeError if one cannot be read.
    """
    folder = workspace_path / DEVCONTAINER.parent
    files = [folder / DEVCONTAINER.name] + sorted(folder.glob(f'*/{DEVCONTAINER.name}'))
    return [path for path in files
            if path.is_file() and 'auro' in (read_jsonc(path).get('customizations') or {})]


def check_task1_scenario(scenarios):
    """Check that exactly one scenario is of task1 (one without a task parameter is of task2)."""
    task1 = [scenario_name(s, i) for i, s in enumerate(scenarios, 1)
             if scenario_task(s) == 'task1']
    if len(task1) != 1:
        fail('Scenarios: there must be exactly one scenario of task1 (with "task": "task1" in its '
             f'parameters), found {len(task1)}' + (f": {', '.join(task1)}." if task1 else '.'))
        return False
    ok(f'Scenarios: exactly one scenario of task1: {task1[0]}.')
    return True


def check_devcontainers(workspace_path: Path):
    """Check that only one devcontainer.json has "auro" customizations."""
    try:
        found = devcontainers_with_auro(workspace_path)
    except RuntimeError as error:
        fail(f'devcontainer.json: {error}')
        return False
    if len(found) > 1:
        fail('devcontainer.json: "auro" customizations (dependencies and scenarios) are in more '
             f"than one file: {', '.join(str(p.relative_to(workspace_path)) for p in found)}. "
             'Keep them in one.')
        return False
    if found:
        ok(f'devcontainer.json: "auro" customizations only in '
           f'{found[0].relative_to(workspace_path)}.')
    else:
        warn('devcontainer.json: no "auro" customizations (dependencies and scenarios) found.')
    return True


def auro_settings(workspace_path: Path, devcontainer=None):
    """
    Return the "auro" settings: {'path', 'apt': [...], 'pip': [...], 'scenarios': [...]}.

    They are under "customizations": {"auro": {...}} in devcontainer (a path), or else in the one
    devcontainer.json of the workspace that has them; path is that file, or None if there is none.
    Raises RuntimeError if a file cannot be read, or more than one has them.
    """
    if devcontainer:
        path = Path(devcontainer)
    else:
        found = devcontainers_with_auro(workspace_path)
        if len(found) > 1:
            raise RuntimeError('"auro" customizations are in more than one devcontainer.json: '
                               f"{', '.join(str(p) for p in found)}. Keep them in one, or choose "
                               'one with --devcontainer.')
        path = found[0] if found else None
    auro = ((read_jsonc(path).get('customizations') or {}).get('auro') or {}) if path else {}
    settings = {key: auro.get(key, []) or [] for key in ('apt', 'pip', 'scenarios')}
    for key in ('apt', 'pip'):
        if not all(isinstance(p, str) for p in settings[key]):
            raise RuntimeError(f'{path}: customizations.auro.{key} must be a list of strings')
    if not all(isinstance(s, dict) for s in settings['scenarios']):
        raise RuntimeError(f'{path}: customizations.auro.scenarios must be a list of objects')
    settings['path'] = path
    return settings


def scenario_name(scenario, index):
    return scenario.get('name', f'Scenario {index}')


def scenario_parameters(scenario):
    """
    Return a scenario's parameters as launch argument strings: true, not True.

    A scenario without a task parameter is of DEFAULT_SCENARIO_TASK, which it is given
    explicitly, as solution.launch.py's own default may differ.
    """
    def text(value):
        return str(value).lower() if isinstance(value, bool) else str(value)
    parameters = {'task': DEFAULT_SCENARIO_TASK}
    parameters.update({name: text(value)
                       for name, value in (scenario.get('parameters') or {}).items()})
    return parameters


def scenario_task(scenario):
    """The task a scenario runs: its task parameter, or DEFAULT_SCENARIO_TASK."""
    return scenario_parameters(scenario)['task']


def find_scenario(scenarios, key):
    """Return (index, scenario) by number (from 1, as listed) or name (any case)."""
    if key.isdigit() and 1 <= int(key) <= len(scenarios):
        return int(key), scenarios[int(key) - 1]
    for i, scenario in enumerate(scenarios, 1):
        if scenario_name(scenario, i).lower() == key.lower():
            return i, scenario
    raise RuntimeError(f'no scenario {key!r}: give its number (1 to {len(scenarios)}) or name, '
                       'as list-scenarios shows')


def list_scenarios(workspace_path: Path, settings, task=None):
    """Print the scenarios, of a task or all, with whether their launch arguments are valid."""
    scenarios = settings['scenarios']
    shown = [(i, s) for i, s in enumerate(scenarios, 1) if task in (None, scenario_task(s))]
    if not shown:
        warn('No scenarios' + (f' for {task}' if task else '') + '.')
        return True
    try:
        errors = check_argument_values(
            workspace_path, {str(i): scenario_parameters(s) for i, s in shown})
    except RuntimeError as error:
        fail(f'Could not check the scenarios: {error}')
        errors = None
    for i, scenario in shown:
        print(f'  {i}. {scenario_name(scenario, i)} [{scenario_task(scenario)}]')
        print(f"     Description: {scenario.get('description', '(no description)')}")
        print(f'     Parameters: {scenario_parameters(scenario)}')
        if errors is None:
            continue
        if errors[str(i)]:
            print(f'     {RED}❌ Invalid:{RESET}')
            for error in errors[str(i)]:
                print(f'        {error}')
        else:
            print(f'     {GREEN}✅ Valid{RESET}')
    return errors is not None and not any(errors.values())


def run_scenario(workspace_path: Path, settings, key, force=False):
    """Launch solution.launch.py with a scenario's parameters, in place of this process."""
    index, scenario = find_scenario(settings['scenarios'], key)
    parameters = scenario_parameters(scenario)
    errors = check_argument_values(workspace_path, {'scenario': parameters})['scenario']
    if errors:
        fail(f'Scenario {index}, {scenario_name(scenario, index)}, is invalid:')
        for error in errors:
            print(f'     {error}')
        if not force:
            sys.exit(1)
        warn('Launching it anyway (--force).')
    command = ['ros2', 'launch', SOLUTION_PACKAGE, SOLUTION_LAUNCH.name] + [
        f'{name}:={value}' for name, value in parameters.items()]
    print(f'🚀 Scenario {index}, {scenario_name(scenario, index)}: {shlex.join(command)}')
    sys.stdout.flush()
    os.execvp(command[0], command)


def install_dependencies(settings):
    """Install the apt and pip packages that devcontainer.json lists; return whether it worked."""
    ok_so_far = True
    if settings['apt']:
        print(f"Installing apt packages: {' '.join(settings['apt'])}")
        try:
            subprocess.run(['sudo', 'apt-get', 'update'], check=True)
            subprocess.run(['sudo', 'apt-get', 'install', '-y'] + settings['apt'], check=True)
            ok('apt packages installed.')
        except (subprocess.CalledProcessError, FileNotFoundError) as error:
            fail(f'Installing the apt packages failed: {error}')
            ok_so_far = False
    else:
        print('No apt packages to install.')
    if settings['pip']:
        # This Python's pip: in the container, a virtual environment's
        print(f"Installing pip packages: {' '.join(settings['pip'])}")
        try:
            subprocess.run([sys.executable, '-m', 'pip', 'install'] + settings['pip'], check=True)
            ok('pip packages installed.')
        except subprocess.CalledProcessError as error:
            fail(f'Installing the pip packages failed: {error}')
            ok_so_far = False
    else:
        print('No pip packages to install.')
    return ok_so_far


# -----------------------------
# ZIP workspace
# -----------------------------
def zip_workspace(workspace_path: Path, output_file: Path):
    """Create a ZIP archive of a workspace, without build/, install/, log/ and .git/."""
    output_file = output_file.resolve()
    print(f'Zipping workspace {workspace_path} -> {output_file}')
    with zipfile.ZipFile(output_file, 'w', zipfile.ZIP_DEFLATED) as zf:
        for root, _, files in walk(workspace_path):
            for f in files:
                file_path = Path(root) / f
                if file_path != output_file:
                    zf.write(file_path, file_path.relative_to(workspace_path))
    ok('Workspace zipped successfully.')


# -----------------------------
# Main CLI
# -----------------------------
def main():
    parser = argparse.ArgumentParser(
        description=f'AURO2026 assessment workspace tool [version: {VERSION}]')
    subparsers = parser.add_subparsers(dest='command', required=True, help='Subcommands')

    def workspace_arg(p):
        p.add_argument('workspace', nargs='?', default='.',
                       help='Path to the ROS 2 workspace (default: current folder)')

    build_parser = subparsers.add_parser('build', help='Build a ROS 2 workspace')
    workspace_arg(build_parser)
    build_parser.add_argument('--colcon-args', nargs=argparse.REMAINDER,
                              help='Arguments for colcon build (default: --symlink-install)')

    check_parser = subparsers.add_parser(
        'check-submission', help='Check that a workspace is ready to be submitted')
    workspace_arg(check_parser)
    check_parser.add_argument('--upstream', default=UPSTREAM_URL,
                              help=f'Repository of the assessment (default: {UPSTREAM_URL})')
    check_parser.add_argument('--branch', default=UPSTREAM_BRANCH,
                              help=f'Its branch (default: {UPSTREAM_BRANCH})')

    def devcontainer_arg(p):
        p.add_argument('--devcontainer',
                       help='devcontainer.json to read (default: the one of the workspace with '
                       '"auro" customizations)')

    devcontainer_arg(check_parser)

    install_parser = subparsers.add_parser(
        'install-dependencies',
        help='Install the apt and pip packages listed in devcontainer.json')
    workspace_arg(install_parser)
    devcontainer_arg(install_parser)

    scenarios_parser = subparsers.add_parser(
        'list-scenarios', help='List the scenarios in devcontainer.json, and check them')
    workspace_arg(scenarios_parser)
    devcontainer_arg(scenarios_parser)
    scenarios_parser.add_argument('--task', choices=TASKS, help='Only the scenarios of this task')

    run_parser = subparsers.add_parser(
        'run-scenario', help='Launch the solution with a scenario of devcontainer.json')
    run_parser.add_argument('scenario', help='Its number, as list-scenarios shows, or its name')
    run_parser.add_argument('workspace', nargs='?', default='.',
                            help='Path to the ROS 2 workspace (default: current folder)')
    devcontainer_arg(run_parser)
    run_parser.add_argument('--force', action='store_true',
                            help='Launch it even if its launch arguments are invalid')

    check_zip_parser = subparsers.add_parser('check-zip',
                                             help='Check whether a ZIP holds a ROS 2 workspace')
    check_zip_parser.add_argument('zipfile', help='Path to the ZIP file')

    clean_parser = subparsers.add_parser('clean', help='Remove build/, install/ and log/')
    workspace_arg(clean_parser)

    list_launch_parser = subparsers.add_parser('list-launch',
                                               help='List launch files and their arguments')
    workspace_arg(list_launch_parser)

    zip_parser = subparsers.add_parser(
        'zip-workspace', help='Zip a workspace, without build/, install/, log/ and .git/')
    workspace_arg(zip_parser)
    zip_parser.add_argument('output_file', help='Output ZIP file path')

    if argcomplete:
        argcomplete.autocomplete(parser)
    args = parser.parse_args()

    if args.command == 'build':
        colcon_build(workspace_or_exit(args.workspace), args.colcon_args)

    elif args.command == 'clean':
        clean_ros2_workspace(workspace_or_exit(args.workspace))

    elif args.command == 'list-launch':
        launch_info = list_launch_arguments(workspace_or_exit(args.workspace))
        if not launch_info:
            print('No launch files found.')
        for file, arguments in launch_info.items():
            print(f'{file}: {arguments}')

    elif args.command == 'check-zip':
        zip_path = Path(args.zipfile).resolve()
        if not zip_path.is_file() or zip_path.suffix != '.zip':
            fail(f'ZIP file not found: {zip_path}')
            sys.exit(1)
        if not is_ros2_workspace_zip(zip_path):
            fail(f'Not a ROS 2 workspace ZIP: {zip_path}')
            sys.exit(1)
        ok(f'ZIP seems to be a ROS 2 workspace: {zip_path}')

    elif args.command == 'zip-workspace':
        zip_workspace(workspace_or_exit(args.workspace), Path(args.output_file))

    elif args.command in ('install-dependencies', 'list-scenarios', 'run-scenario'):
        workspace_path = workspace_or_exit(args.workspace)
        try:
            settings = auro_settings(workspace_path, args.devcontainer)
            if args.command == 'install-dependencies':
                success = install_dependencies(settings)
            elif args.command == 'list-scenarios':
                print(f"Scenarios in {settings['path'] or 'no devcontainer.json'}:")
                success = list_scenarios(workspace_path, settings, args.task)
            else:
                run_scenario(workspace_path, settings, args.scenario, args.force)
        except RuntimeError as error:
            fail(str(error))
            sys.exit(1)
        if not success:
            sys.exit(1)

    elif args.command == 'check-submission':
        workspace_path = workspace_or_exit(args.workspace)
        try:
            scenarios = auro_settings(workspace_path, args.devcontainer)['scenarios']
        except RuntimeError as error:
            warn(f'Scenarios: not checked: {error}')
            scenarios = []
        results = [
            check_devcontainers(workspace_path),
            check_task1_scenario(scenarios),
            check_launch_arguments(workspace_path),
            check_solution_defaults(workspace_path, scenarios),
            check_gazebo_transport(workspace_path),
            check_assessment_packages(workspace_path, args.upstream, args.branch),
        ]
        print()
        if all(results):
            ok('The workspace passed the checks.')
        else:
            fail('The workspace failed some checks: fix them before submitting it.')
            sys.exit(1)


if __name__ == '__main__':
    main()
