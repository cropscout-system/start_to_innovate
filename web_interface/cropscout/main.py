import datetime
import hashlib
import uuid
from contextlib import asynccontextmanager
from pathlib import Path
from typing import List, Optional

import jwt
# import rclpy
import uvicorn
from fastapi import Depends, FastAPI, HTTPException, status
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import HTMLResponse
from fastapi.security import HTTPAuthorizationCredentials, HTTPBearer
from fastapi.staticfiles import StaticFiles
# from mavros_msgs.msg import Waypoint, WaypointList
from pydantic import BaseModel
# from rclpy.node import Node


# @asynccontextmanager
# async def lifespan(_: FastAPI):
#     yield
#     rclpy.shutdown()


app = FastAPI(title='Agricultural Drone Management System') #, lifespan=lifespan)

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


routes = []
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


# ROS2 Node for drone control
# class DroneControlNode(Node):
    def __init__(self):
        super().__init__('drone_control_node')
        # Publisher for waypoints
        self.waypoint_pub = self.create_publisher(WaypointList, '/mavros/mission/waypoints', 10)
        self.get_logger().info('Drone Control Node initialized')

    def send_waypoints(self, waypoints):
        waypoint_list = WaypointList()

        for idx, wp in enumerate(waypoints):
            waypoint = Waypoint()
            waypoint.frame = 3  # MAV_FRAME_GLOBAL_RELATIVE_ALT
            waypoint.command = 16  # MAV_CMD_NAV_WAYPOINT
            waypoint.is_current = idx == 0
            waypoint.autocontinue = True
            waypoint.param1 = 0.0  # Hold time in seconds
            waypoint.param2 = 2.0  # Acceptance radius in meters
            waypoint.param3 = 0.0  # Pass through waypoint
            waypoint.param4 = 0.0  # Desired yaw angle
            waypoint.x_lat = wp.get('lat')
            waypoint.y_long = wp.get('lng')
            waypoint.z_alt = 30.0  # Fixed altitude of 30 meters

            waypoint_list.waypoints.append(waypoint)

        self.waypoint_pub.publish(waypoint_list)
        self.get_logger().info(f'Published {len(waypoints)} waypoints')
        return True


# rclpy.init()
# drone_node = DroneControlNode()


@app.post('/api/routes', response_model=RouteModel)
async def create_route(route: RouteModel):
    if not route.id:
        route.id = str(uuid.uuid4())
    routes.append(route.dict())
    return route


@app.get('/api/routes', response_model=List[RouteModel])
async def get_routes():
    return routes


@app.get('/api/routes/{route_id}', response_model=RouteModel)
async def get_route(route_id: str):
    for route in routes:
        if route['id'] == route_id:
            return route
    raise HTTPException(status_code=404, detail='Route not found')


@app.put('/api/routes/{route_id}', response_model=RouteModel)
async def update_route(
    route_id: str, updated_route: RouteModel
):
    for i, route in enumerate(routes):
        if route['id'] == route_id:
            updated_data = updated_route.dict()
            updated_data['id'] = route_id
            routes[i] = updated_data
            return updated_data
    raise HTTPException(status_code=404, detail='Route not found')


@app.delete('/api/routes/{route_id}')
async def delete_route(route_id: str):
    for i, route in enumerate(routes):
        if route['id'] == route_id:
            del routes[i]
            return {'message': 'Route deleted successfully'}
    raise HTTPException(status_code=404, detail='Route not found')


@app.post('/api/routes/{route_id}/start')
async def start_mission(route_id: str):
    for route in routes:
        if route['id'] == route_id:
            # success = drone_node.send_waypoints(route['waypoints'])
            success = True
            if success:
                return {'message': 'Mission started successfully'}
            else:
                raise HTTPException(status_code=500, detail='Failed to start mission')
    raise HTTPException(status_code=404, detail='Route not found')


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
