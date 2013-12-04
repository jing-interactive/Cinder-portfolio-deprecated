var os = require('os');
var fs = require('fs');
var sys = require('sys');
var process = require('process');
var image = require('get-image-data');

image('../assets/anim/global_guide/globe_guide.jpg', function(error, info) {
  var height = info.height;
  var width = info.width;
  var data = info.data;

  for (var i = 0, l = data.length; i < l; i += 4) {
	var red = data[i];
	var green = data[i + 1];
	var blue = data[i + 2];
	var alpha = data[i + 3];  
  }
});

document.write("shit1: " + process.cwd());

var images = {};

fs.readFile('../assets/anim/global_guide/globe_guide.jpg', function(err, data){
	if (err) throw err;
	// console.log("shit2: " + data);
});

var scene = new THREE.Scene();
var WIDTH = window.innerWidth,
HEIGHT = window.innerHeight;
renderer = new THREE.WebGLRenderer({antialias:true});
renderer.setSize(WIDTH, HEIGHT);
document.body.appendChild(renderer.domElement);

