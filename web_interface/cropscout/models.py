from pydantic import BaseModel

from cropscout.db.models import Route, Waypoint

__all__ = ['RouteModel', 'UserCredentials', 'PasswordUpdate', 'convert_db_route']


class _WaypointModel(BaseModel):
    id: int
    lat: float
    lng: float
    alt: float


class RouteModel(BaseModel):
    id: str
    name: str
    waypoints: list[_WaypointModel]
    duration_estimate: int | None = None  # minutes


class UserCredentials(BaseModel):
    username: str
    password: str


class PasswordUpdate(BaseModel):
    current_password: str
    new_password: str


def _convert_db_waypoint(db_waypoint: Waypoint, id_: int) -> _WaypointModel:
    return _WaypointModel(
        id=id_, lat=db_waypoint.latitude, lng=db_waypoint.longitude, alt=db_waypoint.altitude
    )


def convert_db_route(db_route: Route, db_waypoints: list[Waypoint]) -> RouteModel:
    fastapi_waypoints = [_convert_db_waypoint(wp, i) for i, wp in enumerate(db_waypoints)]

    return RouteModel(
        id=str(db_route.id),
        name=db_route.name,
        waypoints=fastapi_waypoints,
    )
