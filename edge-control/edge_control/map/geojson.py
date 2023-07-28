from shapely.geometry import Polygon, shape


class Feature:
    def __init__(self, feature: dict):
        self.feature = feature

    def coordinates(self):
        assert self.feature["type"] == "Feature"
        geometry = self.feature["geometry"]
        assert geometry["type"] == "Polygon"
        return geometry["coordinates"]


def to_shape(feature: dict) -> Polygon:
    assert feature["type"] == "Feature"
    geometry = feature["geometry"]
    assert geometry["type"] == "Polygon"
    return shape(geometry).buffer(0)  # buffer to fix overlapping coordinates
