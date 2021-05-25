import requests
import json
URL = "https://maps.googleapis.com/maps/api/elevation/json?path="
API_KEY="AIzaSyDs4rYa3WPPu30NV0K9ysOtxkgWXhb7jF8"
#URL = https://maps.googleapis.com/maps/api/elevation/json?path=36.578581,-118.291994|36.23998,-116.83171&samples=10&key=AIzaSyDs4rYa3WPPu30NV0K9ysOtxkgWXhb7jF8
samples = 10

loc_string = "36.578581,-118.291994|36.23998,-116.83171"

#loc_string = "54.484898756295365,-0.6140756607055664|54.485297638747866,-0.6133460998535156"

data = requests.get(URL + loc_string + "&samples=" + f"{samples}" + "&key=" + API_KEY)#,params="elevation")
data = data.json()['results']

for result in data:
    elevation = result['elevation']
    print(elevation)