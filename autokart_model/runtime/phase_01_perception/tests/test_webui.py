import json
import threading
import time
import urllib.request
import urllib.error
from pathlib import Path
import yaml
from autokart_perception.pipeline import Pipeline
from autokart_perception.synthetic import Synthetic
from autokart_perception.webui import Dashboard,make_server,HTML

CFG=yaml.safe_load((Path(__file__).parents[1]/'config/perception.yaml').read_text())

def test_dashboard_http_and_latest_frame():
    dash=Dashboard(Synthetic(),Pipeline(None,CFG,True))
    server=make_server(dash,'127.0.0.1',0)
    thread=threading.Thread(target=server.serve_forever,daemon=True)
    dash.start();thread.start()
    base=f'http://127.0.0.1:{server.server_address[1]}'
    try:
        with urllib.request.urlopen(base+'/') as response:
            assert b'AutoKart' in response.read()
        deadline=time.monotonic()+3
        while True:
            with urllib.request.urlopen(base+'/api/status') as response:
                status=json.load(response)
            if 'boundary_status' in status['diagnostics'] or time.monotonic()>=deadline:break
            time.sleep(.02)
        assert status['source']=='SYNTHETIC' and status['sequence']>=1
        assert status['toggles']['boundaries'] and status['toggles']['curvature']
        assert status['diagnostics']['boundary_status']=='CANDIDATE'
        assert status['diagnostics']['accepted_gate_candidates']==3
        assert status['diagnostics']['curvature_status']=='PROVISIONAL_CURVATURE'
        request=urllib.request.Request(base+'/api/toggles',data=b'{"delaunay":false}',
            headers={'Content-Type':'application/json'},method='POST')
        with urllib.request.urlopen(request) as response:
            assert json.load(response)['delaunay'] is False
        form=urllib.request.Request(base+'/toggle',data=b'flag=boxes',method='POST')
        with urllib.request.urlopen(form) as response:
            assert response.url==base+'/'
        with urllib.request.urlopen(base+'/api/status') as response:
            assert json.load(response)['toggles']['boxes'] is False
        form=urllib.request.Request(base+'/toggle',data=b'flag=boundaries',method='POST')
        with urllib.request.urlopen(form) as response:
            assert response.url==base+'/'
        with urllib.request.urlopen(base+'/api/status') as response:
            assert json.load(response)['toggles']['boundaries'] is False
        with urllib.request.urlopen(base+'/stream.mjpg',timeout=3) as response:
            chunk=response.read(150)
            assert b'Content-Type: image/jpeg' in chunk
    finally:
        server.shutdown();dash.close();server.server_close();thread.join(timeout=3)

def test_dashboard_js_has_valid_newline_escape():
    assert "+'\\nState: '" in HTML

def test_session_calibration_http_validation_and_status():
    pipeline=Pipeline(None,CFG,True)
    dash=Dashboard(Synthetic(),pipeline)
    server=make_server(dash,'127.0.0.1',0)
    thread=threading.Thread(target=server.serve_forever,daemon=True)
    thread.start();base=f'http://127.0.0.1:{server.server_address[1]}'
    original=CFG['camera_mount']['height_m']
    def post(body,origin=None):
        headers={'Content-Type':'application/json'}
        if origin:headers['Origin']=origin
        return urllib.request.urlopen(urllib.request.Request(base+'/api/calibration',
            data=json.dumps(body).encode(),headers=headers,method='POST'))
    try:
        pose={'height_m':.22,'pitch_down_deg':7.0,'roll_deg':0.0,'yaw_left_deg':0.0}
        with post(pose) as response:calibration=json.load(response)
        assert calibration['mount']['height_m']==.22 and calibration['revision']==1
        assert calibration['status']=='MOUNT_CONFIGURED_UNVERIFIED'
        with urllib.request.urlopen(base+'/api/status') as response:status=json.load(response)
        assert status['calibration']['source']=='session'
        assert status['status']=='CALIBRATION_UPDATED_WAITING_FOR_FRAME'
        assert status['diagnostics']['calibration_status']=='MOUNT_CONFIGURED_UNVERIFIED'
        assert abs(pipeline.mount[2,3]-.22)<1e-9 and CFG['camera_mount']['height_m']==original
        for bad in ({**pose,'height_m':0},{**pose,'height_m':True},{**pose,'pitch_down_deg':90},
                    {**pose,'roll_deg':float('nan')},{'height_m':.2}):
            try:post(bad)
            except urllib.error.HTTPError as exc:assert exc.code==400
            else:assert False,'Invalid calibration was accepted'
        try:post(pose,'http://example.invalid')
        except urllib.error.HTTPError as exc:assert exc.code==403
        else:assert False,'Cross-origin calibration was accepted'
        assert dash.status_json()['calibration']['revision']==1
        assert abs(pipeline.mount[2,3]-.22)<1e-9
    finally:
        server.shutdown();dash.close();server.server_close();thread.join(timeout=3)
