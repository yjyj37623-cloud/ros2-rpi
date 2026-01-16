import json
from flask import Flask
from flask_socketio import emit, join_room, leave_room, SocketIO
from flask_redis import FlaskRedis
from flask import session, redirect, url_for, render_template, request, Blueprint
from flask_wtf import FlaskForm
from wtforms.fields import StringField, SubmitField
from wtforms.validators import DataRequired

app = Flask(__name__)
app.debug = True
app.config['SECRET_KEY'] = 'gjr39dkjn344_!67#'

# main_blueprint = Blueprint('main', __name__)
# app.register_blueprint(main_blueprint)

socketio = SocketIO()
Redis = FlaskRedis(app)
socketio.init_app(app)

# ROOM = 1

# MASTER_STATUS = 0
# SLAVE_STATUS = 0
#
# MASSAGE_FROM_MASTER = []
# MASSAGE_FROM_SLAVE = []

@socketio.on('joined', namespace='/chat')
def joined(message):
    """Sent by clients when they enter a room.
    A status message is broadcast to all people in the room."""
    # room = session.get('room')
    join_room(ROOM)
    name = session.get('name')
    tmp_start = 0
    if name == 'master':
        Redis.lpush('master_status', json.dumps({'is_online': True}))
        emit('message', {'msg': '[LOG]:' + session.get('name') + ' has entered the room.'}, room=ROOM)
        if Redis.llen("slave_status") > 0:
            tmp_start = 1

    elif name == 'slave':
        Redis.lpush('slave_status', json.dumps({'is_online': True}))
        emit('log', {'msg': '[LOG]:' + session.get('name') + ' has entered the room.'}, room=ROOM)
        if Redis.llen("master_status") > 0:
            tmp_start = 1

    if tmp_start:
        emit('log', {'msg': '[LOG]:' + 'Ready to Start.'}, room=ROOM)
        emit('start', {'msg': 'start'}, room=ROOM)


@socketio.on('text', namespace='/chat')
def text(message):
    """Sent by a client when the user entered a new message.
    The message is sent to all people in the room."""
    # room = session.get('room')
    emit('message', {'msg': session.get('name') + ':' + message['msg']}, room=ROOM)


@socketio.on('left', namespace='/chat')
def left(message):
    """Sent by clients when they leave a room.
    A status message is broadcast to all people in the room."""
    # room = session.get('room')
    leave_room(ROOM)
    name = session.get('name')
    if name == 'master':
        Redis.rpop("master_status")
        emit('log', {'msg': '[LOG]:' + session.get('name') + ' has entered the room.'}, room=ROOM)
    elif name == 'slave':
        Redis.rpop("slave_status")
        emit('log', {'msg': '[LOG]:' + session.get('name') + ' has entered the room.'}, room=ROOM)

    emit('stop', {'msg': session.get('name') + ' has left the room.'}, room=ROOM)

@socketio.on('error', namespace='/chat')
def error(message):
    emit('log', {'msg': '[ERROR]:' + session.get('name') + message}, room=ROOM)
    emit('error', {'msg': session.get('name') + message}, room=ROOM)

@socketio.on('to_master', namespace='/chat')
def to_master(message):
    emit('log', {'msg': '[TO_MASTER]:' + message}, room=ROOM)
    emit('master_rec', {'msg': + message}, room=ROOM)

@socketio.on('to_slave', namespace='/chat')
def to_slave(message):
    emit('log', {'msg': '[TO_SLAVE]:' + message}, room=ROOM)
    emit('master_rec', {'msg': + message}, room=ROOM)

class LoginForm(FlaskForm):
    """Accepts a nickname and a room."""
    name = StringField('Name', validators=[DataRequired()])
    room = StringField('Room', validators=[DataRequired()])
    submit = SubmitField('Enter Chatroom')


@app.route('/', methods=['GET', 'POST'])
def index():
    """Login form to enter a room."""
    form = LoginForm()
    if form.validate_on_submit():
        session['name'] = form.name.data
        session['room'] = form.room.data
        return redirect(url_for('.chat'))
    elif request.method == 'GET':
        form.name.data = session.get('name', '')
        form.room.data = session.get('room', '')
    return render_template('index.html', form=form)


@app.route('/chat')
def chat():
    """Chat room. The user's name and room must be stored in
    the session."""
    name = session.get('name', '')
    room = session.get('room', '')
    if name == '' or room == '':
        return redirect(url_for('.index'))
    return render_template('chat.html', name=name, room=room)


def create_app(debug=False):
    return app
