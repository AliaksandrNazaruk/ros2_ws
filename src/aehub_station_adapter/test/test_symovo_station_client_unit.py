import pytest


def test_import_symovo_station_client():
    from aehub_station_adapter.symovo_station_client import SymovoStationClient  # noqa: F401


@pytest.mark.parametrize(
    "payload, ok",
    [
        ('{"name":"S1","pose":{"x":0,"y":0,"theta":0,"map_id":1},"state":"OK"}', True),
        ('{"name":"S1","pose":{"x":0,"y":0,"theta":0,"map_id":1}}', False),
        ("not-json", False),
    ],
)
def test_station_input_json_validation_shape(payload, ok):
    import json

    def parse_obj(text):
        try:
            obj = json.loads(text)
        except Exception:
            return None
        return obj if isinstance(obj, dict) else None

    obj = parse_obj(payload)
    is_ok = bool(obj and "name" in obj and "pose" in obj and "state" in obj)
    assert is_ok == ok

