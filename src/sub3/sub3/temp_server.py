import eventlet
import socketio

# create a Socket.IO server
sio = socketio.Server()

# wrap with a WSGI application
app = socketio.WSGIApp(sio, static_files={
    '/': {'content_type': 'text/html', 'filename': 'index.html'}
})

@sio.event
def connect(sid, environ):
    print('connect ', sid)

# catch-all event handler
@sio.on('*')
def catch_all(event, sid, data):
    print(sid, data)

@sio.event
def sendTime(sid, data):
    print('message ', data)

@sio.event
def streaming(sid, data):
    print('stream: ', data)

# @sio.event
# def my_message(sid, data):
#     print('message ', data)

@sio.event
def disconnect(sid):
    print('disconnect ', sid)

if __name__ == '__main__':
    eventlet.wsgi.server(eventlet.listen(('', 5000)), app)
    