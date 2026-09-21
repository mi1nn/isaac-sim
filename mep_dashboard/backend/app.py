"""Mock dashboard host. No database, ROS2, simulator or streaming connections."""
from pathlib import Path
from fastapi import FastAPI
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles

FRONTEND = Path(__file__).resolve().parent.parent / 'frontend'
app = FastAPI(title='MEP Dashboard · Mock UI', docs_url=None, redoc_url=None)
app.mount('/static', StaticFiles(directory=FRONTEND), name='static')

@app.get('/', include_in_schema=False)
def index():
    return FileResponse(FRONTEND / 'index.html')

@app.get('/health')
def health():
    return {'status': 'ok', 'mode': 'mock'}
