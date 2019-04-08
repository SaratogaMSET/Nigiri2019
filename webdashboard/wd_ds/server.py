from flask import Flask
from flask import render_template
#import logs
import pysftp
from datetime import datetime
import os

cnopts = pysftp.CnOpts()
cnopts.hostkeys = None

fileDownloaded = False

app = Flask(__name__)

@app.route('/', methods=['GET'])
def index():
    return str("""
        <h1>WELCOME TO THE FRC649 WEBDASHBOARD</h1>
        <ul>
        <li><h3><a href=\"/drivercamera\">Driver Camera Feed</a></h3></li>
        <li><h3><a href=\"/visioncamera\">Vision Camera Feed</a></h3></li>
        <li><h3><a href=\"/webdashboard\">Web Dashboard</a></h3></li>
        <li><h3><a href=\"/logs\">Logs and Graphs</a></h3></li>
        </ul>
    """)

@app.route('/webdashboard', methods=['GET'])
def webdashboard():
    return str("")

@app.route('/drivercamera', methods=['GET'])
def drivercamera():
    return str("<html><img src=\"http://10.6.49.17/mjpg/1/video.mjpg\" style=\"display:flex; width: 100%; height: 100%; object-fit: contain;\"/></html>")

@app.route('/logs', methods=['GET'])
def logs():
    files = [] # [filename, last modified, filesize]
    global sftp
    try:
        sftp;
    except:
        sftp =  pysftp.Connection('10.6.49.2', port=22, username='lvuser', password='', cnopts=cnopts)
    sftp.cwd("/home/lvuser/log/")
    logfile_list = sftp.listdir_attr()
    for file in logfile_list:
        files.append([file.filename, file.st_mtime, file.st_size])
    files.sort(key=lambda x: -x[1])
    files = files[:500]
    for file in files:
        file[1] = datetime.fromtimestamp(file[1]).strftime("%m/%d/%y %I:%M:%S %p")
    #print(files)
    return render_template('logs.html', files=files)

@app.route('/reconnect', methods=['GET'])
def reconnect():
    global sftp
    sftp = pysftp.Connection('10.6.49.2', port=22, username='lvuser', password='', cnopts=cnopts)

@app.route("/logs/<string:filename>")
def logfile(filename):
    global fileDownloaded
    fileDownloaded = False
    global sftp
    dir_path = os.path.dirname(os.path.realpath(__file__))
    sftp.get(filename, dir_path + "/logfiles/"+filename, preserve_mtime=True)
    while True:
        continue
    filepath = "../logfiles/"+filename
    return render_template('logfile.html', filename=filename, filepath=filepath)

def main():
    app.run(host='localhost', port=8000, debug=True)

if __name__ == "__main__":
    main()