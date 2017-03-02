
### Plotly over Serial
Graph data from your Arduino with just a serial connection. These sketches connect serially connected Arduinos to Plotly's real-time graphing API with the help of the [Standard Firmata protocal](/firmata/arduino), [Node.js](https://http://nodejs.org/), and [Johnny-Five](/rwaldron/johnny-five).

[![](http://new.tinygrab.com/c751bc2ee22cd3db5a6d3fccb22458d570efc9ac84.png)](http://vimeo.com/93900876)


### Quickstart
1 - Install node.js: [http://nodejs.org/download/](http://nodejs.org/download/)

2 - Launch the Arduino IDE and upload the `Standard Firmata` sketch. It's included in your Arduino IDE under `File` → `Examples` → `Firmata` → `StandardFirmata`.

3 - Once uploaded, close the Arduino IDE. We're done with it. Your computer will now communicate with your Arduino over serial with node.js via the Standard Firmata protocol. Nice!

4 - Open your terminal, create a project folder, move into it:

```bash
$ mkdir plotly_project
$ cd plotly_project
```
Or, if your on a Windows machine, open up the `Node.js command prompt` and enter:

```bash
> md plotly_project
> cd plotly_project
```

5 - Use the Node Package Manager (npm) to install two libraries. node.js is awesome and will only install these modules in your project directory. It's easy to keep things modular and organized!

```bash
$ npm install plotly
$ npm install johnny-five
```

6 - Download the [example script](https://raw.githubusercontent.com/plotly/arduino-api/master/plotly_streaming_serial/simple.js) into your project folder and run it with node:

```bash
$ node simple.js
1399607731073 Device(s) /dev/cu.usbmodem1421 
1399607734347 Connected /dev/cu.usbmodem1421 
1399607734347 Repl Initialized 
>> View your streaming graph here: http://plot.ly/~streaming-demos/6/
```

or, on Windows it'll look something like:
```bash
C:\...\plotly_node> node simple.js
1400090004506 Device(s) COM4
1400090007789 Connected COM4
1400090007789 Repl Initialized
>> View your streaming graph here: http://plot.ly/~streaming-demos/6/
```


7 - Grab the url that is printed out and view your live-updating graph in plotly! Here is an example url: [https://plot.ly/~streaming-demos/6](https://plot.ly/~streaming-demos/6)

### Projects

See our [workshop page](https://plot.ly/workshop) for step-by-step instructions.

- DHT22 Temperature and Humidity sensor: [http://plot.ly/workshop/arduino-dht22/](http://plot.ly/workshop/arduino-dht22/)

- Analog Light Sensor: [http://plot.ly/workshop/arduino-analoglight/](http://plot.ly/workshop/arduino-analoglight/)

- ML8511 UV Sensor: [http://plot.ly/workshop/arduino-uvsensor/](http://plot.ly/workshop/arduino-uvsensor/)

- Air Quality Sensor: [http://plot.ly/workshop/arduino-airquality/](http://plot.ly/workshop/arduino-airquality/)

- Water Flow Sensor: [http://plot.ly/workshop/arduino-waterflow/](http://plot.ly/workshop/arduino-waterflow/)

- TMP36 Temperature Sensor: [http://plot.ly/workshop/arduino-tmp36/](http://plot.ly/workshop/arduino-tmp36/)



### Get in touch
- <chris@plot.ly>
- [@plotlygraphs](https://twitter.com/plotlygraphs)



