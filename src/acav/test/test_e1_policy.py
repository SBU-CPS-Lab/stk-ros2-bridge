import json
from acav.campaign import RunSpec, build_e1_convention
from acav.campaign_runner import write_run_config


def test_only_e1_forced_provider_gets_explicit_ablation(tmp_path):
    control, treatment = build_e1_convention().runs
    for run, expected in [(control, 'strict'), (treatment, 'e1_readback_ablation'),
                          (RunSpec(experiment='E6', scheduler='dynamics_aware',
                                   execution_mode='lockstep', attitude_provider='wrong'), 'strict')]:
        path = write_run_config(run, str(tmp_path / 'out.json'))
        payload = json.loads(open(path, encoding='utf-8').read())
        assert payload['attitude_mismatch_policy'] == expected
        assert payload['experiment'] == run.experiment
        assert payload['attitude_provider'] == run.attitude_provider
