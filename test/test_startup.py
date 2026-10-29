"""startup/_setup.sh finds configs by the same rules as config.find_config. Needs bash."""
from pathlib import Path
import shutil
import subprocess

import pytest

SETUP = Path(__file__).resolve().parent.parent / 'startup' / '_setup.sh'

pytestmark = pytest.mark.skipif(shutil.which('bash') is None, reason='needs bash')


@pytest.fixture()
def tree(tmp_path):
    """A copy of _setup.sh in startup/, next to a config/ like the repo's, default b.yaml."""
    (tmp_path / 'startup').mkdir()
    text = SETUP.read_text()
    default = next(line for line in text.splitlines() if line.startswith('DEFAULT_CONFIG_NAME='))
    (tmp_path / 'startup' / '_setup.sh').write_text(
        text.replace(default, 'DEFAULT_CONFIG_NAME="b.yaml"'))
    for rel in ('config/a.yaml', 'config/tether/b.yaml', 'config/mrs/dup.yaml',
                'config/tether/dup.yaml', 'here/c.yaml'):
        (tmp_path / rel).parent.mkdir(parents=True, exist_ok=True)
        (tmp_path / rel).write_text('{}\n')
    return tmp_path


def source_setup(tree, *args):
    """Source _setup.sh from `tree` with args. Returns (exit code, HERO_SIM_CONFIG, stderr)."""
    script = 'source startup/_setup.sh "$@" > /dev/null && printf %s "$HERO_SIM_CONFIG"'
    done = subprocess.run(['bash', '-c', script, 'bash', *args], cwd=tree,
                          capture_output=True, text=True,
                          env={'PATH': '/usr/bin:/bin', 'HOME': str(tree)})
    return done.returncode, done.stdout, done.stderr


@pytest.mark.parametrize('arg, found', [
    ('a.yaml', 'config/a.yaml'),           # on top of config/
    ('b.yaml', 'config/tether/b.yaml'),    # in a folder, by name
    ('tether/b.yaml', 'config/tether/b.yaml'),
    ('mrs/dup.yaml', 'config/mrs/dup.yaml'),
    ('here/c.yaml', 'here/c.yaml'),        # an existing path, relative to where it runs
])
def test_setup_finds_configs(tree, arg, found):
    code, path, err = source_setup(tree, arg)
    assert code == 0, err
    assert path == str(tree / found)


def test_setup_defaults_and_absolute_paths(tree):
    assert source_setup(tree)[1] == str(tree / 'config' / 'tether' / 'b.yaml')
    assert source_setup(tree, str(tree / 'here' / 'c.yaml'))[1] == str(tree / 'here' / 'c.yaml')


@pytest.mark.parametrize('arg, message', [
    ('dup.yaml', 'more than one config folder'),
    ('nope.yaml', 'config not found'),
    ('mrs/b.yaml', 'config not found'),
    ('/no/such/file.yaml', 'config not found: /no/such/file.yaml'),
])
def test_setup_errors(tree, arg, message):
    code, _, err = source_setup(tree, arg)
    assert code != 0 and message in err
