import datetime
import hashlib
import uuid
from pathlib import Path
from typing import List, Optional

import jwt
import uvicorn
from fastapi import Depends, FastAPI, HTTPException, status
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import HTMLResponse
from fastapi.security import HTTPAuthorizationCredentials, HTTPBearer
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel
from sqlmodel import Session, SQLModel, create_engine, select

import models

# Database setup
DATABASE_URL = "sqlite:///cropscout.db"
engine = create_engine(DATABASE_URL)

SQLModel.metadata.drop_all(engine)  # FOR TESTING
SQLModel.metadata.create_all(engine)


def get_session():
    with Session(engine) as session:
        yield session


app = FastAPI(title='Agricultural Drone Management System')

# Configure CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=['*'],  # for development only
    allow_credentials=True,
    allow_methods=['*'],
    allow_headers=['*'],
)

# Constants
SECRET_KEY = 'your-secret-key-change-in-production'
ALGORITHM = 'HS256'
TOKEN_EXPIRE_MINUTES = 60


# Models
class WaypointModel(BaseModel):
    id: int
    lat: float
    lng: float


class RouteModel(BaseModel):
    id: str
    name: str
    waypoints: List[WaypointModel]
    duration_estimate: Optional[int] = None  # minutes


class UserCredentials(BaseModel):
    username: str
    password: str


class PasswordUpdate(BaseModel):
    current_password: str
    new_password: str


# Replaced with a database in the future
users = {'admin': {'password_hash': hashlib.sha256('admin'.encode()).hexdigest(), 'is_admin': True}}


def get_password_hash(password: str) -> str:
    return hashlib.sha256(password.encode()).hexdigest()


def verify_password(plain_password: str, hashed_password: str) -> bool:
    return get_password_hash(plain_password) == hashed_password


def create_access_token(data: dict):
    to_encode = data.copy()
    expire = datetime.datetime.utcnow() + datetime.timedelta(minutes=TOKEN_EXPIRE_MINUTES)
    to_encode.update({'exp': expire})
    encoded_jwt = jwt.encode(to_encode, SECRET_KEY, algorithm=ALGORITHM)
    return encoded_jwt


def decode_token(token: str):
    try:
        payload = jwt.decode(token, SECRET_KEY, algorithms=[ALGORITHM])
        return payload
    except jwt.PyJWTError:
        return None


security = HTTPBearer()


async def get_current_user(credentials: HTTPAuthorizationCredentials = Depends(security)):
    token = credentials.credentials
    payload = decode_token(token)
    if payload is None:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail='Invalid authentication credentials',
            headers={'WWW-Authenticate': 'Bearer'},
        )
    return payload.get('sub')


# Helper functions to convert between API models and database models
def db_waypoint_to_api_waypoint(db_waypoint, index):
    return WaypointModel(
        id=index,
        lat=db_waypoint.latitude,
        lng=db_waypoint.longitude
    )


def db_route_to_api_route(db_route, db_waypoints):
    # Convert database route to API route model
    api_waypoints = [
        db_waypoint_to_api_waypoint(wp, i)
        for i, wp in enumerate(db_waypoints)
    ]

    return RouteModel(
        id=str(db_route.id),
        name=getattr(db_route, 'name', f"Route {db_route.id}"),  # Default name if not present
        waypoints=api_waypoints
    )


@app.post('/api/routes', response_model=RouteModel)
async def create_route(route: RouteModel, db: Session = Depends(get_session), username: str = Depends(get_current_user)):
    # Create a new database route
    db_route = models.Route()
    db_route.name = route.name
    db.add(db_route)
    db.commit()
    db.refresh(db_route)

    # Create waypoints
    for wp in route.waypoints:
        db_waypoint = models.Waypoint(
            latitude=wp.lat,
            longitude=wp.lng,
            altitude=30.0  # Default altitude
        )
        db.add(db_waypoint)
        db.commit()
        db.refresh(db_waypoint)

        # Link waypoint to route
        db.add(models.RouteWaypoint(
            route_id=db_route.id,
            waypoint_id=db_waypoint.id
        ))

    db.commit()

    # Return the created route
    return db_route_to_api_route(db_route, db_route.waypoints)


@app.get('/api/routes', response_model=List[RouteModel])
async def get_routes(db: Session = Depends(get_session), username: str = Depends(get_current_user)):
    # Get all routes from database
    routes_query = select(models.Route)
    db_routes = db.exec(routes_query).all()

    result = []
    for db_route in db_routes:
        result.append(db_route_to_api_route(db_route, db_route.waypoints))

    return result


@app.get('/api/routes/{route_id}', response_model=RouteModel)
async def get_route(route_id: str, db: Session = Depends(get_session), username: str = Depends(get_current_user)):
    # Convert string ID to UUID
    try:
        route_uuid = uuid.UUID(route_id)
    except ValueError:
        raise HTTPException(status_code=400, detail="Invalid route ID format")

    # Get route from database
    db_route = db.get(models.Route, route_uuid)
    if not db_route:
        raise HTTPException(status_code=404, detail='Route not found')

    return db_route_to_api_route(db_route, db_route.waypoints)


@app.post('/api/routes/{route_id}/start')
async def start_mission(
        route_id: str, db: Session = Depends(get_session), username: str = Depends(get_current_user)
):
    # Convert string ID to UUID
    try:
        route_uuid = uuid.UUID(route_id)
    except ValueError:
        raise HTTPException(status_code=400, detail="Invalid route ID format")

    # Get route from database
    db_route = db.get(models.Route, route_uuid)
    if not db_route:
        raise HTTPException(status_code=404, detail='Route not found')

    # Create a new mission
    mission = models.Mission(route_id=route_uuid)
    db.add(mission)
    db.commit()
    db.refresh(mission)

    # Create planned visits for each waypoint
    for order, waypoint in enumerate(db_route.waypoints):
        visit = models.WaypointVisit(
            mission_id=mission.id,
            waypoint_id=waypoint.id,
            order=order,
            status=models.VisitStatus.PLANNED
        )
        db.add(visit)

    db.commit()

    # Here you would send waypoints to the drone
    # success = drone_node.send_waypoints(db_route.waypoints)
    success = True

    if success:
        return {'message': 'Mission started successfully', 'mission_id': str(mission.id)}
    else:
        # Clean up the mission if it failed to start
        db.delete(mission)
        db.commit()
        raise HTTPException(status_code=500, detail='Failed to start mission')


@app.get('/api/missions/current/waypoints', response_model=List[dict])
async def get_waypoints(
        db: Session = Depends(get_session),
        # username: str = Depends(get_current_user)
        # NO AUTHENTICATION WHILE TESTING
):
    # Find all waypoint visits with PLANNED status
    planned_visits_query = select(models.WaypointVisit).where(
        models.WaypointVisit.status == models.VisitStatus.PLANNED
    ).order_by(models.WaypointVisit.order)

    planned_visits = db.exec(planned_visits_query).all()

    if not planned_visits:
        return []

    # Group by mission
    mission_waypoints = {}
    for visit in planned_visits:
        if visit.mission_id not in mission_waypoints:
            mission_waypoints[visit.mission_id] = []

        # Get the waypoint data
        waypoint = db.get(models.Waypoint, visit.waypoint_id)
        if not waypoint:
            continue

        mission_waypoints[visit.mission_id].append({
            'flight_mission_id': str(visit.mission_id),
            'point_id': str(waypoint.id),
            'point_visit_id': str(visit.id),
            'order': visit.order,
            'status': visit.status,
            'latitude': waypoint.latitude,
            'longitude': waypoint.longitude,
            'photo_altitude': waypoint.altitude
        })

        # Update status to IN_PROGRESS
        visit.status = models.VisitStatus.IN_PROGRESS

    # Commit the status changes
    db.commit()

    # Return waypoints from all active missions
    # (In a real system, you might want to filter for a specific mission)
    result = []
    for mission_id, waypoints in mission_waypoints.items():
        result.extend(waypoints)

    return result


@app.post('/api/auth/login')
async def login(credentials: UserCredentials):
    username = credentials.username
    password = credentials.password

    if username not in users:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail='Incorrect username or password',
            headers={'WWW-Authenticate': 'Bearer'},
        )

    if not verify_password(password, users[username]['password_hash']):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail='Incorrect username or password',
            headers={'WWW-Authenticate': 'Bearer'},
        )

    access_token = create_access_token({'sub': username})
    return {'access_token': access_token, 'token_type': 'bearer'}


@app.post('/api/auth/change-password')
async def change_password(
        password_update: PasswordUpdate, username: str = Depends(get_current_user)
):
    if not verify_password(password_update.current_password, users[username]['password_hash']):
        raise HTTPException(status_code=400, detail='Current password is incorrect')

    users[username]['password_hash'] = get_password_hash(password_update.new_password)
    return {'message': 'Password updated successfully'}


STATIC_ROOT = Path('static')


@app.get('/', response_class=HTMLResponse)
async def get_home():
    with Path.open(STATIC_ROOT / 'index.html') as f:
        content = f.read()
    return content


app.mount('/static', StaticFiles(directory=str(STATIC_ROOT.absolute())), name='static')


def main():
    uvicorn.run(app, log_level='trace', host='0.0.0.0', port=8000)


if __name__ == '__main__':
    main()
