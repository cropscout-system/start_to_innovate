from datetime import UTC, datetime, tzinfo
from enum import StrEnum, auto
from uuid import UUID, uuid4

from sqlmodel import Field, Relationship, Session, SQLModel, create_engine


class VisitStatus(StrEnum):
    PLANNED = auto()
    IN_PROGRESS = auto()
    COMPLETED = auto()
    FAILED = auto()


class RouteWaypoint(SQLModel, table=True):
    route_id: UUID = Field(foreign_key='route.id', primary_key=True)
    waypoint_id: UUID = Field(foreign_key='waypoint.id', primary_key=True)


class Waypoint(SQLModel, table=True):
    id: UUID = Field(default_factory=uuid4, primary_key=True)
    created_at: datetime = Field(default_factory=lambda: datetime.now(UTC))

    latitude: float = Field(alias='lat')
    longitude: float = Field(alias='lon')
    altitude: float = Field(alias='alt')

    routes: list['Route'] = Relationship(back_populates='waypoints', link_model=RouteWaypoint)
    visits: list['WaypointVisit'] = Relationship(back_populates='waypoint')


class WaypointVisit(SQLModel, table=True):
    __tablename__ = 'waypoint_visits'

    id: UUID = Field(default_factory=uuid4, primary_key=True)
    order: int = Field(default=0)
    visited_at: datetime = Field(default=datetime.fromtimestamp(0, UTC))
    status: VisitStatus

    mission_id: UUID = Field(foreign_key='mission.id')
    mission: 'Mission' = Relationship(back_populates='waypoint_status')

    waypoint_id: UUID = Field(foreign_key='waypoint.id')
    waypoint: Waypoint = Relationship(back_populates='visits')


class Route(SQLModel, table=True):
    id: UUID = Field(default_factory=uuid4, primary_key=True)
    created_at: datetime = Field(default_factory=lambda: datetime.now(UTC))
    name: str = Field(default="")  # Add name field with empty string default

    waypoints: list[Waypoint] = Relationship(back_populates='routes', link_model=RouteWaypoint)
    missions: list['Mission'] = Relationship(back_populates='route')


class Mission(SQLModel, table=True):
    id: UUID = Field(default_factory=uuid4, primary_key=True)

    waypoint_status: list[WaypointVisit] = Relationship(back_populates='mission')
    start_time: datetime = Field(default_factory=lambda: datetime.now(UTC))
    end_time: datetime | None = Field(default=None)
    emergency_return: bool = Field(default=False)

    route_id: UUID = Field(foreign_key='route.id')
    route: Route = Relationship(back_populates='missions')
