"""Boundary regression for SDK-to-MoveIt joint limits, without any robot client."""
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml

CONTROL = Path(__file__).resolve().parents[1] / 'runtime/Control'
PACKAGE = CONTROL / 'ros2_ws/src/d1_pick_place'
sys.path.insert(0, str(PACKAGE))
from d1_pick_place.sdk_executor import LIMITS, Mapping, NAMES


@pytest.mark.parametrize('index', range(6))
def test_exact_sdk_endpoints_are_inside_urdf_and_outside_values_are_rejected(index):
    config = yaml.safe_load((PACKAGE / 'config/first_pick.yaml').read_text())
    mapping = Mapping.from_candidate(config['hardware_mapping_candidate'])
    model = ET.parse(PACKAGE / 'config/d1.urdf').getroot()
    limit = model.find(f"joint[@name='{NAMES[index]}']/limit")
    lower, upper = float(limit.get('lower')), float(limit.get('upper'))
    converted = []
    for sdk_boundary in LIMITS[index]:
        sdk = np.zeros(7)
        sdk[index] = sdk_boundary
        angle = mapping.to_model(sdk)[index]
        converted.append(angle)
        assert lower <= angle <= upper
    assert lower == min(converted)
    assert upper == max(converted)
    assert not lower <= lower - np.deg2rad(.0001) <= upper
    assert not lower <= upper + np.deg2rad(.0001) <= upper


def test_current_negative_ninety_degree_start_is_preserved_without_clamping():
    config = yaml.safe_load((PACKAGE / 'config/first_pick.yaml').read_text())
    mapping = Mapping.from_candidate(config['hardware_mapping_candidate'])
    sdk = [1.1, -90., 88.1, 6.9, 13.2, -6.8, 6.5]
    model = mapping.to_model(sdk)
    assert model[1] == -np.pi / 2
    assert np.allclose(mapping.to_sdk(model), sdk[:6], atol=1e-12)
    root = ET.parse(PACKAGE / 'config/d1.urdf').getroot()
    for name, angle in zip(NAMES, model):
        limit = root.find(f"joint[@name='{name}']/limit")
        assert float(limit.get('lower')) <= angle <= float(limit.get('upper'))
