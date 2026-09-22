"""Dashboard host. Mock UI data; the only live input is the Astrobee camera image (CAMERA 3,
ROS 2 -> `astrobee_feed.py`). No database connection."""
from contextlib import asynccontextmanager
from pathlib import Path
from fastapi import FastAPI, Response
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles

from .astrobee_feed import AstrobeeFeed

FRONTEND = Path(__file__).resolve().parent.parent / 'frontend'
feed = AstrobeeFeed()


@asynccontextmanager
async def lifespan(_app):
    feed.start()
    yield
    feed.stop()

app = FastAPI(title='MEP Dashboard · Mock UI', docs_url=None, redoc_url=None, lifespan=lifespan)
app.mount('/static', StaticFiles(directory=FRONTEND), name='static')

@app.get('/', include_in_schema=False)
def index():
    return FileResponse(FRONTEND / 'index.html')

@app.get('/health')
def health():
    return {'status': 'ok', 'mode': 'mock'}

@app.get('/api/camera/astrobee.jpg', include_in_schema=False)
def astrobee_frame():
    """Newest Astrobee camera frame (JPEG); 204 while no recent frame has arrived."""
    jpeg = feed.latest()
    if jpeg is None:
        return Response(status_code=204, headers={'Cache-Control': 'no-store'})
    return Response(content=jpeg, media_type='image/jpeg', headers={'Cache-Control': 'no-store'})
