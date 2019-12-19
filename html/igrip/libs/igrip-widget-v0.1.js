(function (cjs, an) {

var p; // shortcut to reference prototypes
var lib={};var ss={};var img={};
lib.ssMetadata = [
		{name:"igrip_widget_v0.1_atlas_", frames: [[0,0,930,930],[1039,0,87,88],[1128,0,87,88],[932,0,50,191],[1191,90,8,238],[932,193,8,238],[984,0,53,172],[770,932,768,721],[0,932,768,768],[1125,90,31,68],[1158,90,31,68],[1260,41,31,68],[1217,41,41,68],[1039,90,41,68],[1082,90,41,68],[1217,0,80,39]]}
];


// symbols:



(lib.CachedBmp_16 = function() {
	this.initialize(ss["igrip_widget_v0.1_atlas_"]);
	this.gotoAndStop(0);
}).prototype = p = new cjs.Sprite();



(lib.CachedBmp_15 = function() {
	this.initialize(ss["igrip_widget_v0.1_atlas_"]);
	this.gotoAndStop(1);
}).prototype = p = new cjs.Sprite();



(lib.CachedBmp_14 = function() {
	this.initialize(ss["igrip_widget_v0.1_atlas_"]);
	this.gotoAndStop(2);
}).prototype = p = new cjs.Sprite();



(lib.CachedBmp_13 = function() {
	this.initialize(ss["igrip_widget_v0.1_atlas_"]);
	this.gotoAndStop(3);
}).prototype = p = new cjs.Sprite();



(lib.CachedBmp_12 = function() {
	this.initialize(ss["igrip_widget_v0.1_atlas_"]);
	this.gotoAndStop(4);
}).prototype = p = new cjs.Sprite();



(lib.CachedBmp_11 = function() {
	this.initialize(ss["igrip_widget_v0.1_atlas_"]);
	this.gotoAndStop(5);
}).prototype = p = new cjs.Sprite();



(lib.CachedBmp_10 = function() {
	this.initialize(ss["igrip_widget_v0.1_atlas_"]);
	this.gotoAndStop(6);
}).prototype = p = new cjs.Sprite();



(lib.CachedBmp_9 = function() {
	this.initialize(ss["igrip_widget_v0.1_atlas_"]);
	this.gotoAndStop(7);
}).prototype = p = new cjs.Sprite();



(lib.CachedBmp_8 = function() {
	this.initialize(ss["igrip_widget_v0.1_atlas_"]);
	this.gotoAndStop(8);
}).prototype = p = new cjs.Sprite();



(lib.CachedBmp_7 = function() {
	this.initialize(ss["igrip_widget_v0.1_atlas_"]);
	this.gotoAndStop(9);
}).prototype = p = new cjs.Sprite();



(lib.CachedBmp_6 = function() {
	this.initialize(ss["igrip_widget_v0.1_atlas_"]);
	this.gotoAndStop(10);
}).prototype = p = new cjs.Sprite();



(lib.CachedBmp_5 = function() {
	this.initialize(ss["igrip_widget_v0.1_atlas_"]);
	this.gotoAndStop(11);
}).prototype = p = new cjs.Sprite();



(lib.CachedBmp_4 = function() {
	this.initialize(ss["igrip_widget_v0.1_atlas_"]);
	this.gotoAndStop(12);
}).prototype = p = new cjs.Sprite();



(lib.CachedBmp_3 = function() {
	this.initialize(ss["igrip_widget_v0.1_atlas_"]);
	this.gotoAndStop(13);
}).prototype = p = new cjs.Sprite();



(lib.CachedBmp_2 = function() {
	this.initialize(ss["igrip_widget_v0.1_atlas_"]);
	this.gotoAndStop(14);
}).prototype = p = new cjs.Sprite();



(lib.CachedBmp_1 = function() {
	this.initialize(ss["igrip_widget_v0.1_atlas_"]);
	this.gotoAndStop(15);
}).prototype = p = new cjs.Sprite();
// helper functions:

function mc_symbol_clone() {
	var clone = this._cloneProps(new this.constructor(this.mode, this.startPosition, this.loop));
	clone.gotoAndStop(this.currentFrame);
	clone.paused = this.paused;
	clone.framerate = this.framerate;
	return clone;
}

function getMCSymbolPrototype(symbol, nominalBounds, frameBounds) {
	var prototype = cjs.extend(symbol, cjs.MovieClip);
	prototype.clone = mc_symbol_clone;
	prototype.nominalBounds = nominalBounds;
	prototype.frameBounds = frameBounds;
	return prototype;
	}


(lib.an_TextInput = function(options) {
	this._element = new $.an.TextInput(options);
	this._el = this._element.create();
	var $this = this;
	this.addEventListener('added', function() {
		$this._lastAddedFrame = $this.parent.currentFrame;
		$this._element.attach($('#dom_overlay_container'));
	});
}).prototype = p = new cjs.MovieClip();
p.nominalBounds = new cjs.Rectangle(0,0,100,22);

p._tick = _tick;
p._handleDrawEnd = _handleDrawEnd;
p._updateVisibility = _updateVisibility;



(lib.Symbol22 = function(mode,startPosition,loop) {
	this.initialize(mode,startPosition,loop,{});

	// Layer_1
	this.instance = new lib.CachedBmp_16();
	this.instance.setTransform(0,0,0.4946,0.4946);

	this.timeline.addTween(cjs.Tween.get(this.instance).wait(1));

	this._renderFirstFrame();

}).prototype = getMCSymbolPrototype(lib.Symbol22, new cjs.Rectangle(0,0,460,460), null);


(lib.Symbol17copy = function(mode,startPosition,loop) {
	this.initialize(mode,startPosition,loop,{});

	// Layer_1
	this.instance = new lib.CachedBmp_15();
	this.instance.setTransform(-2.15,-2.15,0.5,0.5);

	this.timeline.addTween(cjs.Tween.get(this.instance).wait(1));

	this._renderFirstFrame();

}).prototype = p = new cjs.MovieClip();
p.nominalBounds = new cjs.Rectangle(-2.1,-2.1,43.5,44);


(lib.Symbol17 = function(mode,startPosition,loop) {
	this.initialize(mode,startPosition,loop,{});

	// Layer_1
	this.instance = new lib.CachedBmp_14();
	this.instance.setTransform(-2.15,-2.15,0.5,0.5);

	this.timeline.addTween(cjs.Tween.get(this.instance).wait(1));

	this._renderFirstFrame();

}).prototype = p = new cjs.MovieClip();
p.nominalBounds = new cjs.Rectangle(-2.1,-2.1,43.5,44);


(lib.Symbol13 = function(mode,startPosition,loop) {
	this.initialize(mode,startPosition,loop,{});

	// Layer_2
	this.instance = new lib.CachedBmp_8();
	this.instance.setTransform(-2,-2,0.5,0.5);

	this.timeline.addTween(cjs.Tween.get(this.instance).wait(1));

	this._renderFirstFrame();

}).prototype = p = new cjs.MovieClip();
p.nominalBounds = new cjs.Rectangle(-2,-2,384,384);


(lib.an_Label = function(options) {
	this._element = new $.an.Label(options);
	this._el = this._element.create();
	var $this = this;
	this.addEventListener('added', function() {
		$this._lastAddedFrame = $this.parent.currentFrame;
		$this._element.attach($('#dom_overlay_container'));
	});
}).prototype = p = new cjs.MovieClip();
p.nominalBounds = new cjs.Rectangle(0,0,100,22);

p._tick = _tick;
p._handleDrawEnd = _handleDrawEnd;
p._updateVisibility = _updateVisibility;



(lib.Symbol16copy = function(mode,startPosition,loop) {
	this.initialize(mode,startPosition,loop,{});

	// Layer_1
	this.limitInput = new lib.an_TextInput({'id': 'limitInput', 'value':'', 'disabled':false, 'visible':true, 'class':'ui-textinput'});

	this.limitInput.setTransform(-13.6,55.05,0.9059,0.8728,89.8247,0,0,50.4,10.7);

	this.instance = new lib.CachedBmp_13();
	this.instance.setTransform(-2.5,5.25,0.5,0.5);

	this.instance_1 = new lib.CachedBmp_12();
	this.instance_1.setTransform(-2,-2,0.5,0.5);

	this.timeline.addTween(cjs.Tween.get({}).to({state:[{t:this.instance_1},{t:this.instance},{t:this.limitInput}]}).wait(1));

	this._renderFirstFrame();

}).prototype = getMCSymbolPrototype(lib.Symbol16copy, new cjs.Rectangle(-24,-2,46.5,119), null);


(lib.Symbol16 = function(mode,startPosition,loop) {
	this.initialize(mode,startPosition,loop,{});

	// value_
	this.valueInput = new lib.an_TextInput({'id': 'valueInput', 'value':'', 'disabled':false, 'visible':true, 'class':'ui-textinput'});

	this.valueInput.setTransform(14.6,59.7,0.9065,0.8841,-90,0,0,49.9,11.1);

	this.instance = new lib.CachedBmp_10();
	this.instance.setTransform(-25.85,7.5,0.5,0.5);

	this.timeline.addTween(cjs.Tween.get({}).to({state:[{t:this.instance},{t:this.valueInput}]}).wait(1));

	// Layer_1
	this.instance_1 = new lib.CachedBmp_11();
	this.instance_1.setTransform(-2,-2,0.5,0.5);

	this.timeline.addTween(cjs.Tween.get(this.instance_1).wait(1));

	this._renderFirstFrame();

}).prototype = getMCSymbolPrototype(lib.Symbol16, new cjs.Rectangle(-25.8,-2,50.5,119), null);


(lib.Symbol14copy = function(mode,startPosition,loop) {
	this.initialize(mode,startPosition,loop,{});

	// timeline functions:
	this.frame_0 = function() {
		this.stop();
	}

	// actions tween:
	this.timeline.addTween(cjs.Tween.get(this).call(this.frame_0).wait(600));

	// Layer_4 (mask)
	var mask = new cjs.Shape();
	mask._off = true;
	var mask_graphics_0 = new cjs.Graphics().moveTo(29.3,196.8).lineTo(87.3,96.6).lineTo(88.4,97.3).lineTo(32.2,198.5).curveTo(30.8,197.8,29.3,196.8).closePath();
	var mask_graphics_1 = new cjs.Graphics().moveTo(25.8,195.6).lineTo(85.4,96.4).lineTo(87.7,97.7).lineTo(31.5,198.9).curveTo(28.7,197.4,25.8,195.6).closePath();
	var mask_graphics_2 = new cjs.Graphics().moveTo(23,193.9).lineTo(84.3,95.7).curveTo(86,96.8,87.7,97.7).lineTo(31.5,198.9).curveTo(27.2,196.6,23,193.9).closePath();
	var mask_graphics_3 = new cjs.Graphics().moveTo(20.3,192.1).lineTo(83.3,95).lineTo(87.7,97.7).lineTo(31.5,198.9).curveTo(25.7,195.8,20.3,192.1).closePath();
	var mask_graphics_4 = new cjs.Graphics().moveTo(17.4,190.3).lineTo(82.2,94.3).curveTo(84.9,96.1,87.7,97.7).lineTo(31.5,198.9).curveTo(24.4,194.9,17.4,190.3).closePath();
	var mask_graphics_5 = new cjs.Graphics().moveTo(14.7,188.4).lineTo(81.1,93.5).curveTo(84.3,95.8,87.7,97.7).lineTo(31.5,198.9).curveTo(22.9,194.1,14.7,188.4).closePath();
	var mask_graphics_6 = new cjs.Graphics().moveTo(12,186.5).lineTo(80.1,92.8).curveTo(83.8,95.5,87.7,97.7).lineTo(31.5,198.9).curveTo(21.4,193.3,12,186.5).closePath();
	var mask_graphics_7 = new cjs.Graphics().moveTo(9.3,184.5).lineTo(79.1,92).curveTo(83.1,95.1,87.7,97.7).lineTo(31.5,198.9).curveTo(20,192.5,9.3,184.5).closePath();
	var mask_graphics_8 = new cjs.Graphics().moveTo(6.7,182.5).lineTo(78,91.2).curveTo(82.6,94.9,87.7,97.7).lineTo(31.5,198.9).curveTo(18.5,191.7,6.7,182.5).closePath();
	var mask_graphics_9 = new cjs.Graphics().moveTo(4.2,180.5).lineTo(77,90.5).curveTo(82,94.6,87.7,97.7).lineTo(31.5,198.9).curveTo(17,190.9,4.2,180.5).closePath();
	var mask_graphics_10 = new cjs.Graphics().moveTo(1.6,178.3).lineTo(76.1,89.6).curveTo(81.5,94.2,87.7,97.7).lineTo(31.5,198.9).curveTo(15.5,190.1,1.6,178.3).closePath();
	var mask_graphics_11 = new cjs.Graphics().moveTo(-0.8,176.3).lineTo(75,88.8).curveTo(81,93.9,87.7,97.7).lineTo(31.5,198.9).curveTo(14.2,189.3,-0.8,176.3).closePath();
	var mask_graphics_12 = new cjs.Graphics().moveTo(-3.4,174).lineTo(74.1,88).curveTo(80.3,93.6,87.7,97.7).lineTo(31.5,198.9).curveTo(12.7,188.4,-3.4,174).closePath();
	var mask_graphics_13 = new cjs.Graphics().moveTo(-5.8,171.8).lineTo(73.1,87.1).curveTo(79.7,93.2,87.7,97.7).lineTo(31.5,198.9).curveTo(11.2,187.6,-5.8,171.8).closePath();
	var mask_graphics_14 = new cjs.Graphics().moveTo(-8.3,169.5).lineTo(72.2,86.2).curveTo(79.2,93,87.7,97.7).lineTo(31.5,198.9).curveTo(9.7,186.8,-8.3,169.5).closePath();
	var mask_graphics_15 = new cjs.Graphics().moveTo(-10.6,167.2).lineTo(71.3,85.3).curveTo(78.7,92.7,87.7,97.7).lineTo(31.5,198.9).curveTo(8.2,186,-10.6,167.2).closePath();
	var mask_graphics_16 = new cjs.Graphics().moveTo(-12.9,164.9).lineTo(70.4,84.4).curveTo(78.1,92.3,87.7,97.7).lineTo(31.5,198.9).curveTo(6.7,185.2,-12.9,164.9).closePath();
	var mask_graphics_17 = new cjs.Graphics().moveTo(-15.2,162.4).lineTo(69.4,83.5).curveTo(77.4,92,87.7,97.7).lineTo(31.5,198.9).curveTo(5.3,184.4,-15.2,162.4).closePath();
	var mask_graphics_18 = new cjs.Graphics().moveTo(-17.4,160).lineTo(68.6,82.5).curveTo(76.9,91.6,87.7,97.7).lineTo(31.5,198.9).curveTo(3.8,183.6,-17.4,160).closePath();
	var mask_graphics_19 = new cjs.Graphics().moveTo(-19.7,157.4).lineTo(67.8,81.6).curveTo(76.4,91.3,87.7,97.7).lineTo(31.5,198.9).curveTo(2.3,182.8,-19.7,157.4).closePath();
	var mask_graphics_20 = new cjs.Graphics().moveTo(-21.7,155).lineTo(67,80.5).curveTo(75.7,91.1,87.7,97.7).lineTo(31.5,198.9).curveTo(0.8,181.9,-21.7,155).closePath();
	var mask_graphics_21 = new cjs.Graphics().moveTo(-23.9,152.4).lineTo(66.1,79.6).curveTo(75.1,90.7,87.7,97.7).lineTo(31.5,198.9).curveTo(-0.7,181.1,-23.9,152.4).closePath();
	var mask_graphics_22 = new cjs.Graphics().moveTo(-25.9,149.8).lineTo(65.4,78.6).curveTo(74.6,90.4,87.7,97.7).lineTo(31.5,198.9).curveTo(-2.2,180.2,-25.9,149.8).closePath();
	var mask_graphics_23 = new cjs.Graphics().moveTo(-27.9,147.3).lineTo(64.6,77.5).curveTo(73.9,90.1,87.7,97.7).lineTo(31.5,198.9).curveTo(-3.7,179.4,-27.9,147.3).closePath();
	var mask_graphics_24 = new cjs.Graphics().moveTo(-30,144.6).lineTo(63.8,76.4).curveTo(73.4,89.7,87.7,97.7).lineTo(31.5,198.9).curveTo(-5.2,178.6,-30,144.6).closePath();
	var mask_graphics_25 = new cjs.Graphics().moveTo(-31.8,141.9).lineTo(63.1,75.5).curveTo(72.8,89.4,87.7,97.7).lineTo(31.5,198.9).curveTo(-6.7,177.7,-31.8,141.9).closePath();
	var mask_graphics_26 = new cjs.Graphics().moveTo(-33.7,139.1).lineTo(62.3,74.4).curveTo(72.2,89,87.7,97.7).lineTo(31.5,198.9).curveTo(-8.3,176.9,-33.7,139.1).closePath();
	var mask_graphics_27 = new cjs.Graphics().moveTo(-35.5,136.3).lineTo(61.6,73.3).curveTo(71.6,88.8,87.7,97.7).lineTo(31.5,198.9).curveTo(-9.8,176,-35.5,136.3).closePath();
	var mask_graphics_28 = new cjs.Graphics().moveTo(-37.3,133.6).lineTo(60.9,72.3).curveTo(71.1,88.4,87.7,97.7).lineTo(31.5,198.9).curveTo(-11.3,175.2,-37.3,133.6).closePath();
	var mask_graphics_29 = new cjs.Graphics().moveTo(-39,130.8).lineTo(60.2,71.2).curveTo(70.4,88.1,87.7,97.7).lineTo(31.5,198.9).curveTo(-12.9,174.4,-39,130.8).closePath();
	var mask_graphics_30 = new cjs.Graphics().moveTo(-40.7,127.9).lineTo(59.6,70.1).curveTo(69.9,87.7,87.7,97.7).lineTo(31.5,198.9).curveTo(-14.4,173.4,-40.7,127.9).closePath();
	var mask_graphics_31 = new cjs.Graphics().moveTo(-42.3,125.1).lineTo(58.9,68.9).curveTo(69.2,87.4,87.7,97.7).lineTo(31.5,198.9).curveTo(-16,172.6,-42.3,125.1).closePath();
	var mask_graphics_32 = new cjs.Graphics().moveTo(-43.9,122.1).lineTo(58.3,67.8).curveTo(68.6,87,87.7,97.7).lineTo(31.5,198.9).curveTo(-17.5,171.7,-43.9,122.1).closePath();
	var mask_graphics_33 = new cjs.Graphics().moveTo(-45.4,119.2).lineTo(57.7,66.6).curveTo(68,86.7,87.7,97.7).lineTo(31.5,198.9).curveTo(-19.1,170.8,-45.4,119.2).closePath();
	var mask_graphics_34 = new cjs.Graphics().moveTo(-46.9,116.3).lineTo(57.1,65.5).curveTo(67.3,86.3,87.7,97.7).lineTo(31.5,198.9).curveTo(-20.7,169.9,-46.9,116.3).closePath();
	var mask_graphics_35 = new cjs.Graphics().moveTo(-48.4,113.3).lineTo(56.6,64.3).curveTo(66.7,86.1,87.7,97.7).lineTo(31.5,198.9).curveTo(-22.4,169.1,-48.4,113.3).closePath();
	var mask_graphics_36 = new cjs.Graphics().moveTo(-49.7,110.2).lineTo(56,63.2).curveTo(66.1,85.7,87.7,97.7).lineTo(31.5,198.9).curveTo(-23.9,168.1,-49.7,110.2).closePath();
	var mask_graphics_37 = new cjs.Graphics().moveTo(-51.1,107.2).lineTo(55.5,62).curveTo(65.5,85.4,87.7,97.7).lineTo(31.5,198.9).curveTo(-25.5,167.3,-51.1,107.2).closePath();
	var mask_graphics_38 = new cjs.Graphics().moveTo(-52.3,104.1).lineTo(55.1,60.7).curveTo(64.8,85,87.7,97.7).lineTo(31.5,198.9).curveTo(-27.1,166.4,-52.3,104.1).closePath();
	var mask_graphics_39 = new cjs.Graphics().moveTo(-53.5,101.1).lineTo(54.6,59.5).curveTo(64.2,84.6,87.7,97.7).lineTo(31.5,198.9).curveTo(-28.7,165.4,-53.5,101.1).closePath();
	var mask_graphics_40 = new cjs.Graphics().moveTo(-54.7,98).lineTo(54.1,58.3).curveTo(63.6,84.3,87.7,97.7).lineTo(31.5,198.9).curveTo(-30.5,164.6,-54.7,98).closePath();
	var mask_graphics_41 = new cjs.Graphics().moveTo(-55.8,94.9).lineTo(53.7,57.1).curveTo(62.9,83.9,87.7,97.7).lineTo(31.5,198.9).curveTo(-32.1,163.7,-55.8,94.9).closePath();
	var mask_graphics_42 = new cjs.Graphics().moveTo(-56.8,91.8).lineTo(53.3,55.9).curveTo(62.3,83.6,87.7,97.7).lineTo(31.5,198.9).curveTo(-33.7,162.7,-56.8,91.8).closePath();
	var mask_graphics_43 = new cjs.Graphics().moveTo(-57.9,88.5).lineTo(52.9,54.6).curveTo(61.6,83.2,87.7,97.7).lineTo(31.5,198.9).curveTo(-35.4,161.8,-57.9,88.5).closePath();
	var mask_graphics_44 = new cjs.Graphics().moveTo(-58.8,85.4).lineTo(52.5,53.4).curveTo(60.9,82.8,87.7,97.7).lineTo(31.5,198.9).curveTo(-37.1,160.8,-58.8,85.4).closePath();
	var mask_graphics_45 = new cjs.Graphics().moveTo(-26,149.7).curveTo(-49.7,119.4,-59.6,82.1).lineTo(52.2,52.2).curveTo(56,66.7,65.2,78.5).curveTo(74.6,90.4,87.7,97.7).lineTo(31.5,198.9).curveTo(-2.2,180.2,-26,149.7).closePath();
	var mask_graphics_46 = new cjs.Graphics().moveTo(-27,148.5).curveTo(-50.9,117.2,-60.4,79).lineTo(51.8,51).curveTo(55.6,65.9,65,78.1).curveTo(74.3,90.3,87.7,97.7).lineTo(31.5,198.9).curveTo(-2.9,179.8,-27,148.5).closePath();
	var mask_graphics_47 = new cjs.Graphics().moveTo(-27.9,147.1).curveTo(-52.2,115,-61.2,75.8).lineTo(51.6,49.8).curveTo(55.1,64.9,64.4,77.5).curveTo(73.9,90.1,87.7,97.7).lineTo(31.5,198.9).curveTo(-3.7,179.4,-27.9,147.1).closePath();
	var mask_graphics_48 = new cjs.Graphics().moveTo(-29,145.8).curveTo(-53.4,112.7,-61.9,72.5).lineTo(51.3,48.4).curveTo(54.7,64.1,64.2,77).curveTo(73.6,89.9,87.7,97.7).lineTo(31.5,198.9).curveTo(-4.5,179,-29,145.8).closePath();
	var mask_graphics_49 = new cjs.Graphics().moveTo(-30,144.6).curveTo(-54.6,110.6,-62.6,69.3).lineTo(51,47.2).curveTo(54.1,63.3,63.8,76.4).curveTo(73.4,89.7,87.7,97.7).lineTo(31.5,198.9).curveTo(-5.2,178.6,-30,144.6).closePath();
	var mask_graphics_50 = new cjs.Graphics().moveTo(-30.9,143.2).curveTo(-55.8,108.3,-63.3,66).lineTo(50.8,46).curveTo(53.7,62.4,63.4,75.9).curveTo(73.1,89.6,87.7,97.7).lineTo(31.5,198.9).curveTo(-6,178.2,-30.9,143.2).closePath();
	var mask_graphics_51 = new cjs.Graphics().moveTo(-31.8,141.9).curveTo(-56.9,106,-63.8,62.8).lineTo(50.6,44.6).curveTo(53.2,61.6,62.9,75.5).curveTo(72.8,89.4,87.7,97.7).lineTo(31.5,198.9).curveTo(-6.7,177.7,-31.8,141.9).closePath();
	var mask_graphics_52 = new cjs.Graphics().moveTo(-32.8,140.5).curveTo(-58.1,103.7,-64.2,59.5).lineTo(50.4,43.4).curveTo(52.8,60.6,62.7,74.8).curveTo(72.6,89.2,87.7,97.7).lineTo(31.5,198.9).curveTo(-7.5,177.3,-32.8,140.5).closePath();
	var mask_graphics_53 = new cjs.Graphics().moveTo(-33.7,139.1).curveTo(-59.2,101.4,-64.8,56.3).lineTo(50.2,42.1).curveTo(52.4,59.7,62.3,74.3).curveTo(72.2,89,87.7,97.7).lineTo(31.5,198.9).curveTo(-8.3,176.9,-33.7,139.1).closePath();
	var mask_graphics_54 = new cjs.Graphics().moveTo(-34.7,137.7).curveTo(-60.3,99.1,-65,52.9).lineTo(50.1,40.8).curveTo(52,58.8,61.9,73.9).curveTo(71.9,88.9,87.7,97.7).lineTo(31.5,198.9).curveTo(-9,176.4,-34.7,137.7).closePath();
	var mask_graphics_55 = new cjs.Graphics().moveTo(-35.5,136.3).curveTo(-61.2,96.8,-65.4,49.6).lineTo(49.9,39.6).curveTo(51.6,57.9,61.6,73.3).curveTo(71.6,88.8,87.7,97.7).lineTo(31.5,198.9).curveTo(-9.8,176,-35.5,136.3).closePath();
	var mask_graphics_56 = new cjs.Graphics().moveTo(-36.5,135).curveTo(-62.3,94.3,-65.7,46.4).lineTo(49.8,38.3).curveTo(51.2,56.9,61.2,72.8).curveTo(71.3,88.6,87.7,97.7).lineTo(31.5,198.9).curveTo(-10.6,175.6,-36.5,135).closePath();
	var mask_graphics_57 = new cjs.Graphics().moveTo(-37.3,133.6).curveTo(-63.3,92,-65.8,43).lineTo(49.8,37).curveTo(50.8,56,60.9,72.1).curveTo(71.1,88.4,87.7,97.7).lineTo(31.5,198.9).curveTo(-11.3,175.2,-37.3,133.6).closePath();
	var mask_graphics_58 = new cjs.Graphics().moveTo(-38.2,132.1).curveTo(-64.2,89.6,-66,39.7).lineTo(49.7,35.7).curveTo(50.4,55.1,60.5,71.6).curveTo(70.7,88.2,87.7,97.7).lineTo(31.5,198.9).curveTo(-12.1,174.8,-38.2,132.1).closePath();
	var mask_graphics_59 = new cjs.Graphics().moveTo(-39,130.8).curveTo(-65.2,87.1,-66.1,36.4).lineTo(49.7,34.5).curveTo(50.1,54.2,60.2,71.2).curveTo(70.4,88.1,87.7,97.7).lineTo(31.5,198.9).curveTo(-12.9,174.4,-39,130.8).closePath();
	var mask_graphics_60 = new cjs.Graphics().moveTo(-40,129.3).curveTo(-66.1,84.8,-66.1,33.1).lineTo(49.7,33.1).curveTo(49.7,53.3,59.8,70.6).curveTo(70.1,88,87.7,97.7).lineTo(31.5,198.9).curveTo(-13.7,173.8,-40,129.3).closePath();
	var mask_graphics_61 = new cjs.Graphics().moveTo(-40.8,127.9).curveTo(-67.1,82.4,-66.1,29.9).lineTo(49.7,31.8).curveTo(49.3,52.3,59.6,69.9).curveTo(69.9,87.7,87.7,97.7).lineTo(31.5,198.9).curveTo(-14.4,173.4,-40.8,127.9).closePath();
	var mask_graphics_62 = new cjs.Graphics().moveTo(-41.6,126.4).curveTo(-67.9,80,-66,26.5).lineTo(49.7,30.5).curveTo(49,51.4,59.2,69.4).curveTo(69.4,87.6,87.7,97.7).lineTo(31.5,198.9).curveTo(-15.2,173,-41.6,126.4).closePath();
	var mask_graphics_63 = new cjs.Graphics().moveTo(-42.4,125.1).curveTo(-68.7,77.5,-65.8,23.2).lineTo(49.8,29.2).curveTo(48.7,50.4,58.9,68.9).curveTo(69.2,87.4,87.7,97.7).lineTo(31.5,198.9).curveTo(-16,172.6,-42.4,125.1).closePath();
	var mask_graphics_64 = new cjs.Graphics().moveTo(-43.2,123.6).curveTo(-69.5,75,-65.7,19.8).lineTo(49.8,28).curveTo(48.3,49.4,58.6,68.3).curveTo(68.9,87.3,87.7,97.7).lineTo(31.5,198.9).curveTo(-16.8,172.2,-43.2,123.6).closePath();
	var mask_graphics_65 = new cjs.Graphics().moveTo(-43.9,122.1).curveTo(-70.3,72.5,-65.4,16.6).lineTo(49.9,26.6).curveTo(48.1,48.4,58.3,67.6).curveTo(68.6,87,87.7,97.7).lineTo(31.5,198.9).curveTo(-17.5,171.7,-43.9,122.1).closePath();
	var mask_graphics_66 = new cjs.Graphics().moveTo(-44.7,120.6).curveTo(-71,70.1,-65,13.3).lineTo(50.1,25.4).curveTo(47.8,47.5,57.9,67.1).curveTo(68.2,86.9,87.7,97.7).lineTo(31.5,198.9).curveTo(-18.3,171.2,-44.7,120.6).closePath();
	var mask_graphics_67 = new cjs.Graphics().moveTo(-45.5,119.1).curveTo(-71.8,67.5,-64.8,10).lineTo(50.2,24.2).curveTo(47.5,46.5,57.7,66.6).curveTo(68,86.7,87.7,97.7).lineTo(31.5,198.9).curveTo(-19.1,170.8,-45.5,119.1).closePath();
	var mask_graphics_68 = new cjs.Graphics().moveTo(-46.2,117.8).curveTo(-72.5,65.1,-64.2,6.7).lineTo(50.4,22.8).curveTo(47.2,45.6,57.4,66).curveTo(67.7,86.6,87.7,97.7).lineTo(31.5,198.9).curveTo(-19.9,170.4,-46.2,117.8).closePath();
	var mask_graphics_69 = new cjs.Graphics().moveTo(-47,116.1).curveTo(-73.2,62.5,-63.8,3.5).lineTo(50.6,21.6).curveTo(47,44.5,57.1,65.3).curveTo(67.3,86.3,87.7,97.7).lineTo(31.5,198.9).curveTo(-20.7,169.9,-47,116.1).closePath();
	var mask_graphics_70 = new cjs.Graphics().moveTo(-47.7,114.6).curveTo(-73.8,59.9,-63.3,0.2).lineTo(50.8,20.2).curveTo(46.7,43.5,56.9,64.8).curveTo(67,86.2,87.7,97.7).lineTo(31.5,198.9).curveTo(-21.6,169.5,-47.7,114.6).closePath();
	var mask_graphics_71 = new cjs.Graphics().moveTo(-48.4,113.1).curveTo(-74.4,57.4,-62.6,-3).lineTo(51,19).curveTo(46.4,42.6,56.6,64.3).curveTo(66.7,86.1,87.7,97.7).lineTo(31.5,198.9).curveTo(-22.4,169.1,-48.4,113.1).closePath();
	var mask_graphics_72 = new cjs.Graphics().moveTo(-49,111.7).curveTo(-74.9,54.8,-61.9,-6.3).lineTo(51.3,17.8).curveTo(46.3,41.5,56.3,63.7).curveTo(66.5,85.9,87.7,97.7).lineTo(31.5,198.9).curveTo(-23.2,168.7,-49,111.7).closePath();
	var mask_graphics_73 = new cjs.Graphics().moveTo(-49.7,110.2).curveTo(-75.6,52.2,-61.2,-9.5).lineTo(51.6,16.5).curveTo(46,40.6,56,63).curveTo(66.1,85.7,87.7,97.7).lineTo(31.5,198.9).curveTo(-23.9,168.1,-49.7,110.2).closePath();
	var mask_graphics_74 = new cjs.Graphics().moveTo(-50.4,108.7).curveTo(-76,49.6,-60.4,-12.8).lineTo(51.8,15.2).curveTo(45.7,39.6,55.8,62.5).curveTo(65.8,85.5,87.7,97.7).lineTo(31.5,198.9).curveTo(-24.7,167.7,-50.4,108.7).closePath();
	var mask_graphics_75 = new cjs.Graphics().moveTo(-51.1,107.2).curveTo(-76.5,47.1,-59.6,-15.9).lineTo(52.2,14).curveTo(45.6,38.5,55.5,62).curveTo(65.5,85.4,87.7,97.7).lineTo(31.5,198.9).curveTo(-25.5,167.3,-51.1,107.2).closePath();
	var mask_graphics_76 = new cjs.Graphics().moveTo(-51.8,105.6).curveTo(-77.1,44.5,-58.8,-19.2).lineTo(52.5,12.8).curveTo(45.5,37.6,55.2,61.3).curveTo(65.1,85.1,87.7,97.7).lineTo(31.5,198.9).curveTo(-26.3,166.8,-51.8,105.6).closePath();
	var mask_graphics_77 = new cjs.Graphics().moveTo(-52.3,104.1).curveTo(-77.5,41.9,-57.9,-22.3).lineTo(52.9,11.6).curveTo(45.2,36.5,55,60.7).curveTo(64.8,85,87.7,97.7).lineTo(31.5,198.9).curveTo(-27.1,166.4,-52.3,104.1).closePath();
	var mask_graphics_78 = new cjs.Graphics().moveTo(-53,102.6).curveTo(-77.9,39.2,-56.8,-25.5).lineTo(53.3,10.4).curveTo(45.1,35.6,54.8,60.2).curveTo(64.6,84.8,87.7,97.7).lineTo(31.5,198.9).curveTo(-27.9,166,-53,102.6).closePath();
	var mask_graphics_79 = new cjs.Graphics().moveTo(-53.5,101).curveTo(-78.3,36.6,-55.8,-28.6).lineTo(53.7,9.1).curveTo(44.9,34.5,54.6,59.5).curveTo(64.2,84.6,87.7,97.7).lineTo(31.5,198.9).curveTo(-28.7,165.4,-53.5,101).closePath();
	var mask_graphics_80 = new cjs.Graphics().moveTo(-54.1,99.5).curveTo(-78.6,34.1,-54.7,-31.8).lineTo(54.1,7.9).curveTo(44.8,33.5,54.3,59).curveTo(63.9,84.4,87.7,97.7).lineTo(31.5,198.9).curveTo(-29.5,165,-54.1,99.5).closePath();
	var mask_graphics_81 = new cjs.Graphics().moveTo(-54.7,98).curveTo(-79,31.4,-53.5,-34.9).lineTo(54.6,6.7).curveTo(44.7,32.4,54.1,58.3).curveTo(63.6,84.3,87.7,97.7).lineTo(31.5,198.9).curveTo(-30.5,164.6,-54.7,98).closePath();
	var mask_graphics_82 = new cjs.Graphics().moveTo(-55.3,96.4).curveTo(-79.2,28.6,-52.3,-37.8).lineTo(55.1,5.5).curveTo(44.5,31.4,53.9,57.8).curveTo(63.2,84.2,87.7,97.7).lineTo(31.5,198.9).curveTo(-31.3,164.1,-55.3,96.4).closePath();
	var mask_graphics_83 = new cjs.Graphics().moveTo(-55.8,94.9).curveTo(-79.5,26.1,-51.1,-41).lineTo(55.5,4.3).curveTo(44.5,30.4,53.7,57.1).curveTo(62.9,83.9,87.7,97.7).lineTo(31.5,198.9).curveTo(-32.1,163.7,-55.8,94.9).closePath();
	var mask_graphics_84 = new cjs.Graphics().moveTo(-56.4,93.2).curveTo(-79.7,23.4,-49.7,-43.9).lineTo(56,3).curveTo(44.4,29.3,53.5,56.5).curveTo(62.5,83.8,87.7,97.7).lineTo(31.5,198.9).curveTo(-32.9,163.3,-56.4,93.2).closePath();
	var mask_graphics_85 = new cjs.Graphics().moveTo(-56.9,91.6).curveTo(-79.9,20.7,-48.4,-47.1).lineTo(56.6,2).curveTo(44.3,28.2,53.2,55.9).curveTo(62.3,83.6,87.7,97.7).lineTo(31.5,198.9).curveTo(-33.7,162.7,-56.9,91.6).closePath();
	var mask_graphics_86 = new cjs.Graphics().moveTo(-57.3,90.1).curveTo(-80.1,17.9,-46.9,-50).lineTo(57.1,0.7).curveTo(44.3,27.3,53.1,55.3).curveTo(62,83.4,87.7,97.7).lineTo(31.5,198.9).curveTo(-34.6,162.3,-57.3,90.1).closePath();
	var mask_graphics_87 = new cjs.Graphics().moveTo(-57.9,88.5).curveTo(-80.2,15.2,-45.4,-53).lineTo(57.7,-0.3).curveTo(44.1,26.2,52.8,54.6).curveTo(61.6,83.2,87.7,97.7).lineTo(31.5,198.9).curveTo(-35.4,161.8,-57.9,88.5).closePath();
	var mask_graphics_88 = new cjs.Graphics().moveTo(-58.4,86.9).curveTo(-80.3,12.5,-43.9,-55.9).lineTo(58.3,-1.6).curveTo(44.1,25.1,52.7,54.1).curveTo(61.3,83.1,87.7,97.7).lineTo(31.5,198.9).curveTo(-36.3,161.4,-58.4,86.9).closePath();
	var mask_graphics_89 = new cjs.Graphics().moveTo(-58.8,85.3).curveTo(-80.5,9.8,-42.3,-58.8).lineTo(58.9,-2.6).curveTo(44.1,24,52.5,53.4).curveTo(60.9,82.8,87.7,97.7).lineTo(31.5,198.9).curveTo(-37.1,160.8,-58.8,85.3).closePath();
	var mask_graphics_90 = new cjs.Graphics().moveTo(-39.4,129.8).curveTo(-65.7,85.7,-66.1,34.3).curveTo(-66.4,-17,-40.7,-61.7).lineTo(59.6,-3.7).curveTo(49.5,13.6,49.7,33.5).curveTo(49.8,53.6,60,70.8).curveTo(70.3,88,87.7,97.7).lineTo(31.5,198.9).curveTo(-13.2,174,-39.4,129.8).closePath();
	var mask_graphics_91 = new cjs.Graphics().moveTo(-40.1,128.7).curveTo(-66.4,84,-66.1,32.2).curveTo(-65.8,-19.6,-39,-64.5).lineTo(60.2,-4.9).curveTo(49.8,12.5,49.7,32.7).curveTo(49.5,52.9,59.7,70.4).curveTo(70,87.8,87.7,97.7).lineTo(31.5,198.9).curveTo(-13.8,173.5,-40.1,128.7).closePath();
	var mask_graphics_92 = new cjs.Graphics().moveTo(-40.8,127.9).curveTo(-67.1,82.4,-66.1,29.7).curveTo(-65.2,-22.8,-37.3,-67.4).lineTo(60.9,-6).curveTo(50.1,11.3,49.7,31.8).curveTo(49.3,52.3,59.6,69.9).curveTo(69.9,87.7,87.7,97.7).lineTo(31.5,198.9).curveTo(-14.4,173.4,-40.8,127.9).closePath();
	var mask_graphics_93 = new cjs.Graphics().moveTo(-41.1,126.8).curveTo(-67.5,80.8,-66,27.7).curveTo(-64.4,-25.4,-35.5,-70.1).lineTo(61.6,-7.1).curveTo(50.4,10.4,49.7,30.9).curveTo(49.1,51.7,59.4,69.5).curveTo(69.7,87.6,87.7,97.7).lineTo(31.5,198.9).curveTo(-14.7,173,-41.1,126.8).closePath();
	var mask_graphics_94 = new cjs.Graphics().moveTo(-41.7,125.9).curveTo(-68.1,79.2,-66,25.5).curveTo(-63.8,-28,-33.7,-72.9).lineTo(62.3,-8.2).curveTo(50.6,9.3,49.7,30.1).curveTo(48.9,51,59.2,69.1).curveTo(69.4,87.4,87.7,97.7).lineTo(31.5,198.9).curveTo(-15.3,172.6,-41.7,125.9).closePath();
	var mask_graphics_95 = new cjs.Graphics().moveTo(-42.4,125.1).curveTo(-68.7,77.5,-65.8,23.2).curveTo(-63,-31.1,-31.8,-75.6).lineTo(63.1,-9.3).curveTo(50.9,8.1,49.8,29.2).curveTo(48.7,50.4,58.9,68.9).curveTo(69.2,87.4,87.7,97.7).lineTo(31.5,198.9).curveTo(-16,172.6,-42.4,125.1).closePath();
	var mask_graphics_96 = new cjs.Graphics().moveTo(-42.8,124).curveTo(-69.2,75.9,-65.7,21.1).curveTo(-62.2,-33.7,-30,-78.3).lineTo(63.8,-10.2).curveTo(51.2,7.1,49.8,28.4).curveTo(48.5,49.8,58.7,68.5).curveTo(69,87.3,87.7,97.7).lineTo(31.5,198.9).curveTo(-16.3,172.2,-42.8,124).closePath();
	var mask_graphics_97 = new cjs.Graphics().moveTo(-43.4,123).curveTo(-69.8,74.3,-65.6,19).curveTo(-61.4,-36.2,-27.9,-81).lineTo(64.6,-11.3).curveTo(51.6,6.2,49.9,27.6).curveTo(48.3,49.1,58.5,68.1).curveTo(68.8,87.1,87.7,97.7).lineTo(31.5,198.9).curveTo(-17,171.8,-43.4,123).closePath();
	var mask_graphics_98 = new cjs.Graphics().moveTo(-43.9,122.1).curveTo(-70.3,72.5,-65.4,16.6).curveTo(-60.6,-39.3,-25.9,-83.6).lineTo(65.4,-12.4).curveTo(51.8,4.9,49.9,26.6).curveTo(48.1,48.4,58.3,67.6).curveTo(68.6,87,87.7,97.7).lineTo(31.5,198.9).curveTo(-17.5,171.7,-43.9,122.1).closePath();
	var mask_graphics_99 = new cjs.Graphics().moveTo(-44.3,121).curveTo(-70.7,70.9,-65.2,14.4).curveTo(-59.5,-41.9,-23.9,-86.2).lineTo(66.1,-13.3).curveTo(52.2,3.9,50.1,25.8).curveTo(47.9,47.9,58.2,67.4).curveTo(68.5,86.9,87.7,97.7).lineTo(31.5,198.9).curveTo(-17.8,171.2,-44.3,121).closePath();
	var mask_graphics_100 = new cjs.Graphics().moveTo(-45,120.1).curveTo(-71.3,69.3,-65,12.4).curveTo(-58.7,-44.5,-21.7,-88.8).lineTo(67,-14.3).curveTo(52.5,2.9,50.1,25).curveTo(47.6,47.2,57.9,67).curveTo(68.2,86.7,87.7,97.7).lineTo(31.5,198.9).curveTo(-18.6,171,-45,120.1).closePath();
	var mask_graphics_101 = new cjs.Graphics().moveTo(-45.5,119.1).curveTo(-71.8,67.5,-64.8,10).curveTo(-57.6,-47.5,-19.7,-91.2).lineTo(67.8,-15.4).curveTo(52.9,1.7,50.2,24).curveTo(47.5,46.5,57.7,66.6).curveTo(68,86.7,87.7,97.7).lineTo(31.5,198.9).curveTo(-19.1,170.8,-45.5,119.1).closePath();
	var mask_graphics_102 = new cjs.Graphics().moveTo(-45.8,118.2).curveTo(-72.2,65.9,-64.4,7.9).curveTo(-56.5,-50,-17.4,-93.8).lineTo(68.6,-16.3).curveTo(53.5,0.7,50.4,23.2).curveTo(47.4,45.8,57.5,66.2).curveTo(67.8,86.6,87.7,97.7).lineTo(31.5,198.9).curveTo(-19.4,170.4,-45.8,118.2).closePath();
	var mask_graphics_103 = new cjs.Graphics().moveTo(-46.5,117.1).curveTo(-72.7,64.3,-64.2,5.8).curveTo(-55.5,-52.6,-15.2,-96.2).lineTo(69.4,-17.3).curveTo(53.7,-0.2,50.4,22.4).curveTo(47.1,45.2,57.3,65.8).curveTo(67.6,86.5,87.7,97.7).lineTo(31.5,198.9).curveTo(-20.2,170,-46.5,117.1).closePath();
	var mask_graphics_104 = new cjs.Graphics().moveTo(-47,116.1).curveTo(-73.2,62.5,-63.8,3.5).curveTo(-54.5,-55.6,-12.9,-98.7).lineTo(70.4,-18.2).curveTo(54.3,-1.4,50.6,21.5).curveTo(47,44.5,57.1,65.3).curveTo(67.3,86.3,87.7,97.7).lineTo(31.5,198.9).curveTo(-20.7,169.9,-47,116.1).closePath();
	var mask_graphics_105 = new cjs.Graphics().moveTo(-47.3,115.2).curveTo(-73.6,60.9,-63.4,1.4).curveTo(-53.1,-58,-10.6,-101).lineTo(71.3,-19).curveTo(54.7,-2.4,50.8,20.8).curveTo(46.8,43.9,57,65.1).curveTo(67.3,86.2,87.7,97.7).lineTo(31.5,198.9).curveTo(-21,169.5,-47.3,115.2).closePath();
	var mask_graphics_106 = new cjs.Graphics().moveTo(-47.8,114.1).curveTo(-74,59.1,-63.1,-0.7).curveTo(-52.2,-60.6,-8.3,-103.3).lineTo(72.2,-20).curveTo(55.1,-3.3,50.8,20).curveTo(46.6,43.3,56.7,64.7).curveTo(67,86.1,87.7,97.7).lineTo(31.5,198.9).curveTo(-21.7,169.1,-47.8,114.1).closePath();
	var mask_graphics_107 = new cjs.Graphics().moveTo(-48.4,113.1).curveTo(-74.4,57.4,-62.6,-3.2).curveTo(-50.8,-63.6,-5.8,-105.6).lineTo(73.1,-20.9).curveTo(55.6,-4.5,51,19).curveTo(46.4,42.6,56.6,64.3).curveTo(66.7,86.1,87.7,97.7).lineTo(31.5,198.9).curveTo(-22.4,169.1,-48.4,113.1).closePath();
	var mask_graphics_108 = new cjs.Graphics().moveTo(-48.8,112.2).curveTo(-74.8,55.7,-62.2,-5.2).curveTo(-49.5,-66,-3.4,-107.7).lineTo(74.1,-21.7).curveTo(56.2,-5.5,51.2,18.2).curveTo(46.3,41.9,56.4,63.9).curveTo(66.6,85.9,87.7,97.7).lineTo(31.5,198.9).curveTo(-22.6,168.7,-48.8,112.2).closePath();
	var mask_graphics_109 = new cjs.Graphics().moveTo(-49.3,111.1).curveTo(-75.2,54,-61.8,-7.2).curveTo(-48.4,-68.5,-0.8,-110).lineTo(75,-22.5).curveTo(56.6,-6.4,51.3,17.4).curveTo(46.2,41.2,56.2,63.4).curveTo(66.3,85.7,87.7,97.7).lineTo(31.5,198.9).curveTo(-23.3,168.3,-49.3,111.1).closePath();
	var mask_graphics_110 = new cjs.Graphics().moveTo(-49.7,110.2).curveTo(-75.6,52.2,-61.4,-9.5).curveTo(-47,-71.3,1.6,-112.1).lineTo(76.1,-23.4).curveTo(57.1,-7.5,51.6,16.5).curveTo(46,40.6,56,63).curveTo(66.1,85.7,87.7,97.7).lineTo(31.5,198.9).curveTo(-23.9,168.1,-49.7,110.2).closePath();
	var mask_graphics_111 = new cjs.Graphics().moveTo(-50.1,109.1).curveTo(-75.9,50.6,-60.7,-11.6).curveTo(-45.4,-73.7,4.2,-114.2).lineTo(77,-24.3).curveTo(57.7,-8.5,51.7,15.6).curveTo(45.9,39.9,55.9,62.6).curveTo(65.9,85.5,87.7,97.7).lineTo(31.5,198.9).curveTo(-24.3,167.7,-50.1,109.1).closePath();
	var mask_graphics_112 = new cjs.Graphics().moveTo(-50.7,108).curveTo(-76.3,48.8,-60.3,-13.7).curveTo(-44.2,-76.2,6.7,-116.3).lineTo(78,-25).curveTo(58.2,-9.4,52,14.8).curveTo(45.7,39.2,55.6,62.2).curveTo(65.7,85.4,87.7,97.7).lineTo(31.5,198.9).curveTo(-24.9,167.3,-50.7,108).closePath();
	var mask_graphics_113 = new cjs.Graphics().moveTo(-51.1,107.2).curveTo(-76.5,47.1,-59.7,-16).curveTo(-42.8,-79,9.3,-118.3).lineTo(79.1,-25.8).curveTo(58.7,-10.5,52.1,14).curveTo(45.6,38.5,55.5,62).curveTo(65.5,85.4,87.7,97.7).lineTo(31.5,198.9).curveTo(-25.5,167.3,-51.1,107.2).closePath();
	var mask_graphics_114 = new cjs.Graphics().moveTo(-51.5,106.1).curveTo(-76.9,45.4,-59.1,-18.1).curveTo(-41.1,-81.5,12,-120.3).lineTo(80.1,-26.6).curveTo(59.4,-11.4,52.4,13.2).curveTo(45.5,37.9,55.4,61.6).curveTo(65.4,85.3,87.7,97.7).lineTo(31.5,198.9).curveTo(-25.9,166.9,-51.5,106.1).closePath();
	var mask_graphics_115 = new cjs.Graphics().moveTo(-52,105).curveTo(-77.4,43.7,-58.5,-20.1).curveTo(-39.7,-83.8,14.7,-122.2).lineTo(81.1,-27.3).curveTo(60,-12.4,52.7,12.4).curveTo(45.3,37.2,55.2,61).curveTo(65.1,85,87.7,97.7).lineTo(31.5,198.9).curveTo(-26.6,166.5,-52,105).closePath();
	var mask_graphics_116 = new cjs.Graphics().moveTo(-52.3,104.1).curveTo(-77.5,41.9,-57.9,-22.3).curveTo(-38.2,-86.5,17.4,-124.1).lineTo(82.2,-28.1).curveTo(60.5,-13.5,52.8,11.4).curveTo(45.2,36.5,55,60.7).curveTo(64.8,85,87.7,97.7).lineTo(31.5,198.9).curveTo(-27.1,166.4,-52.3,104.1).closePath();
	var mask_graphics_117 = new cjs.Graphics().moveTo(-52.7,103).curveTo(-77.8,40.2,-57.2,-24.4).curveTo(-36.5,-88.9,20.3,-125.9).lineTo(83.3,-28.8).curveTo(61.2,-14.4,53.1,10.6).curveTo(45.1,35.8,54.8,60.3).curveTo(64.7,84.8,87.7,97.7).lineTo(31.5,198.9).curveTo(-27.5,166,-52.7,103).closePath();
	var mask_graphics_118 = new cjs.Graphics().moveTo(-53.2,101.9).curveTo(-78.2,38.4,-56.6,-26.5).curveTo(-35,-91.2,23,-127.6).lineTo(84.3,-29.5).curveTo(61.9,-15.2,53.3,10).curveTo(44.9,35.1,54.7,59.9).curveTo(64.4,84.7,87.7,97.7).lineTo(31.5,198.9).curveTo(-28.2,165.6,-53.2,101.9).closePath();
	var mask_graphics_119 = new cjs.Graphics().moveTo(-53.5,101).curveTo(-78.3,36.6,-55.8,-28.6).curveTo(-33.3,-93.9,25.8,-129.4).lineTo(85.4,-30.1).curveTo(62.4,-16.3,53.6,9).curveTo(44.9,34.5,54.6,59.5).curveTo(64.2,84.6,87.7,97.7).lineTo(31.5,198.9).curveTo(-28.7,165.4,-53.5,101).closePath();
	var mask_graphics_120 = new cjs.Graphics().moveTo(-53.9,99.9).curveTo(-78.6,34.9,-55,-30.7).curveTo(-31.4,-96.1,28.7,-131).lineTo(86.6,-30.8).curveTo(63.2,-17.3,54,8.2).curveTo(44.8,33.8,54.4,59.1).curveTo(64,84.4,87.7,97.7).lineTo(31.5,198.9).curveTo(-29.1,165,-53.9,99.9).closePath();
	var mask_graphics_121 = new cjs.Graphics().moveTo(-54.3,98.8).curveTo(-78.8,33.1,-54.3,-32.7).curveTo(-29.8,-98.4,31.5,-132.6).lineTo(87.7,-31.5).curveTo(63.8,-18.1,54.1,7.5).curveTo(44.7,33.1,54.1,58.7).curveTo(63.8,84.3,87.7,97.7).lineTo(31.5,198.9).curveTo(-29.8,164.6,-54.3,98.8).closePath();
	var mask_graphics_122 = new cjs.Graphics().moveTo(-55.3,98).curveTo(-79.5,31.4,-54.1,-34.9).curveTo(-28.6,-101,34,-134.3).lineTo(88.3,-32).curveTo(63.9,-19.2,54,6.6).curveTo(44.1,32.4,53.6,58.3).curveTo(63.1,84.3,87.2,97.7).lineTo(31,198.9).curveTo(-31,164.6,-55.3,98).closePath();
	var mask_graphics_123 = new cjs.Graphics().moveTo(-56.2,97.2).curveTo(-80.4,30.3,-53.9,-36.5).curveTo(-27.2,-103.3,36.2,-135.8).lineTo(88.9,-32.7).curveTo(64.1,-20,53.7,6).curveTo(43.4,32,52.9,58).curveTo(62.3,84.2,86.6,97.7).lineTo(30.4,198.9).curveTo(-31.9,164.2,-56.2,97.2).closePath();
	var mask_graphics_124 = new cjs.Graphics().moveTo(-57.2,96.1).curveTo(-81.2,28.5,-53.7,-38.5).curveTo(-26.1,-105.4,38.6,-137.3).lineTo(89.4,-33.2).curveTo(64.2,-20.8,53.5,5.2).curveTo(42.8,31.4,52,57.6).curveTo(61.4,83.9,86,97.7).lineTo(29.8,198.9).curveTo(-33.3,163.7,-57.2,96.1).closePath();
	var mask_graphics_125 = new cjs.Graphics().moveTo(-58.1,94.9).curveTo(-81.8,26.1,-53.4,-41).curveTo(-24.9,-108,41,-138.7).lineTo(90,-33.8).curveTo(64.3,-21.9,53.2,4.3).curveTo(42.2,30.4,51.4,57.1).curveTo(60.6,83.9,85.4,97.7).lineTo(29.2,198.9).curveTo(-34.4,163.7,-58.1,94.9).closePath();
	var mask_graphics_126 = new cjs.Graphics().moveTo(-58.9,94.1).curveTo(-82.5,24.9,-53,-42.7).curveTo(-23.3,-110.2,43.6,-140.1).lineTo(90.6,-34.3).curveTo(64.6,-22.7,53.1,3.6).curveTo(41.5,29.9,50.8,56.8).curveTo(60,83.8,84.9,97.7).lineTo(28.7,198.9).curveTo(-35.2,163.3,-58.9,94.1).closePath();
	var mask_graphics_127 = new cjs.Graphics().moveTo(-60.1,93).curveTo(-83.4,23.1,-52.8,-44.6).curveTo(-22.2,-112.3,46,-141.4).lineTo(91.2,-34.9).curveTo(64.6,-23.5,52.7,2.8).curveTo(40.8,29.2,49.9,56.4).curveTo(59.1,83.6,84.3,97.7).lineTo(28.1,198.9).curveTo(-36.7,162.8,-60.1,93).closePath();
	var mask_graphics_128 = new cjs.Graphics().moveTo(-61,91.6).curveTo(-84,20.7,-52.4,-47.1).curveTo(-20.9,-114.6,48.5,-142.7).lineTo(91.8,-35.3).curveTo(64.8,-24.4,52.5,1.8).curveTo(40.2,28.2,49.1,55.9).curveTo(58.2,83.6,83.7,97.7).lineTo(27.5,198.9).curveTo(-37.8,162.7,-61,91.6).closePath();
	var mask_graphics_129 = new cjs.Graphics().moveTo(-61.8,90.9).curveTo(-84.7,19.6,-52,-48.7).curveTo(-19.2,-116.8,50.8,-143.9).lineTo(92.4,-35.8).curveTo(65,-25.3,52.3,1.3).curveTo(39.6,27.8,48.5,55.6).curveTo(57.5,83.4,83.1,97.7).lineTo(26.9,198.9).curveTo(-38.8,162.3,-61.8,90.9).closePath();
	var mask_graphics_130 = new cjs.Graphics().moveTo(-62.9,89.9).curveTo(-85.6,17.8,-51.9,-50.6).curveTo(-18,-119,53.3,-145.1).lineTo(93,-36.2).curveTo(65.2,-26.1,52,0.5).curveTo(38.8,27.2,47.6,55.2).curveTo(56.6,83.2,82.4,97.7).lineTo(26.2,198.9).curveTo(-40.1,161.9,-62.9,89.9).closePath();
	var mask_graphics_131 = new cjs.Graphics().moveTo(-63.7,88.5).curveTo(-86.1,15.2,-51.4,-53).curveTo(-16.6,-121.3,55.8,-146.2).lineTo(93.6,-36.6).curveTo(65.3,-27,51.8,-0.5).curveTo(38.2,26.2,46.9,54.6).curveTo(55.7,83.2,81.8,97.7).lineTo(25.6,198.9).curveTo(-41.3,161.8,-63.7,88.5).closePath();
	var mask_graphics_132 = new cjs.Graphics().moveTo(-64.6,87.7).curveTo(-86.8,14.2,-50.8,-54.6).curveTo(-14.8,-123.3,58.3,-147.1).lineTo(94.2,-37).curveTo(65.7,-27.7,51.6,-1).curveTo(37.6,25.7,46.3,54.4).curveTo(55,83.1,81.2,97.7).lineTo(25,198.9).curveTo(-42.3,161.4,-64.6,87.7).closePath();
	var mask_graphics_133 = new cjs.Graphics().moveTo(-65.8,86.6).curveTo(-87.7,12.4,-50.6,-56.5).curveTo(-13.5,-125.3,61,-148.2).lineTo(94.8,-37.4).curveTo(65.9,-28.5,51.4,-1.8).curveTo(37,25,45.5,53.8).curveTo(54.1,82.8,80.6,97.7).lineTo(24.4,198.9).curveTo(-43.7,161,-65.8,86.6).closePath();
	var mask_graphics_134 = new cjs.Graphics().moveTo(-66.5,85.3).curveTo(-88.2,9.8,-50.1,-58.8).curveTo(-11.9,-127.5,63.5,-149.2).lineTo(95.4,-37.8).curveTo(66.1,-29.5,51.2,-2.8).curveTo(36.4,24,44.8,53.4).curveTo(53.2,82.8,80,97.7).lineTo(23.8,198.9).curveTo(-44.9,160.8,-66.5,85.3).closePath();
	var mask_graphics_135 = new cjs.Graphics().moveTo(-53.9,119.1).curveTo(-80.1,67.5,-73.1,10).curveTo(-65.9,-47.5,-28,-91.3).curveTo(10.1,-135.1,66.1,-150).lineTo(96.1,-38.1).curveTo(74.3,-32.3,59.4,-15.4).curveTo(44.6,1.7,41.9,24).curveTo(39.2,46.5,49.3,66.6).curveTo(59.6,86.7,79.4,97.7).lineTo(23.2,198.9).curveTo(-27.4,170.8,-53.9,119.1).closePath();
	var mask_graphics_136 = new cjs.Graphics().moveTo(-54.7,118.2).curveTo(-81.1,66,-73.4,8.3).curveTo(-65.7,-49.4,-26.8,-93).curveTo(12.2,-136.6,68.6,-150.8).lineTo(96.7,-38.5).curveTo(74.7,-33,59.6,-16).curveTo(44.4,1,41.4,23.4).curveTo(38.4,45.8,48.6,66.2).curveTo(58.9,86.6,78.8,97.7).lineTo(22.6,198.9).curveTo(-28.3,170.4,-54.7,118.2).closePath();
	var mask_graphics_137 = new cjs.Graphics().moveTo(-55.8,117.8).curveTo(-82,65.1,-73.9,6.7).curveTo(-65.6,-51.5,-25.8,-95).curveTo(14,-138.3,71.3,-151.6).lineTo(97.3,-38.8).curveTo(75.1,-33.7,59.5,-16.7).curveTo(44.1,0.2,40.8,22.8).curveTo(37.7,45.6,47.8,66).curveTo(58.1,86.6,78.2,97.7).lineTo(22,198.9).curveTo(-29.5,170.4,-55.8,117.8).closePath();
	var mask_graphics_138 = new cjs.Graphics().moveTo(-56.8,116.7).curveTo(-83.1,63.4,-74.3,5.1).curveTo(-65.5,-53.3,-24.7,-96.6).curveTo(16,-140,73.8,-152.3).lineTo(98,-39.1).curveTo(75.5,-34.2,59.5,-17.4).curveTo(43.6,-0.5,40.3,22.1).curveTo(36.9,44.9,47,65.6).curveTo(57.3,86.5,77.5,97.7).lineTo(21.3,198.9).curveTo(-30.4,170,-56.8,116.7).closePath();
	var mask_graphics_139 = new cjs.Graphics().moveTo(-57.9,116.1).curveTo(-84,62.5,-74.6,3.5).curveTo(-65.3,-55.6,-23.7,-98.7).curveTo(17.9,-141.6,76.5,-153).lineTo(98.6,-39.3).curveTo(75.7,-34.9,59.6,-18.2).curveTo(43.4,-1.4,39.8,21.5).curveTo(36.1,44.5,46.3,65.3).curveTo(56.4,86.3,76.9,97.7).lineTo(20.7,198.9).curveTo(-31.6,169.9,-57.9,116.1).closePath();
	var mask_graphics_140 = new cjs.Graphics().moveTo(-58.7,115.2).curveTo(-85,60.9,-75,1.7).curveTo(-65,-57.4,-22.4,-100.3).curveTo(20.1,-143.1,79.1,-153.6).lineTo(99.2,-39.6).curveTo(76.3,-35.4,59.6,-18.8).curveTo(43.1,-2.1,39.2,20.9).curveTo(35.4,43.9,45.5,65.1).curveTo(55.8,86.2,76.3,97.7).lineTo(20.1,198.9).curveTo(-32.5,169.6,-58.7,115.2).closePath();
	var mask_graphics_141 = new cjs.Graphics().moveTo(-59.8,114.6).curveTo(-86,59.9,-75.4,0.2).curveTo(-64.8,-59.5,-21.6,-102.2).curveTo(21.7,-144.7,81.7,-154.2).lineTo(99.8,-39.7).curveTo(76.4,-36.1,59.6,-19.6).curveTo(42.8,-2.9,38.6,20.2).curveTo(34.6,43.5,44.7,64.8).curveTo(54.9,86.2,75.6,97.7).lineTo(19.4,198.9).curveTo(-33.7,169.5,-59.8,114.6).closePath();
	var mask_graphics_142 = new cjs.Graphics().moveTo(-60.7,113.7).curveTo(-86.8,58.4,-75.7,-1.4).curveTo(-64.5,-61.3,-20.2,-103.7).curveTo(24.1,-146.1,84.3,-154.6).lineTo(100.5,-40).curveTo(77,-36.6,59.7,-20.1).curveTo(42.5,-3.6,38.2,19.7).curveTo(33.8,43,44,64.5).curveTo(54.1,86.1,75,97.7).lineTo(18.8,198.9).curveTo(-34.6,169.1,-60.7,113.7).closePath();
	var mask_graphics_143 = new cjs.Graphics().moveTo(-61.8,113.1).curveTo(-87.8,57.4,-76,-3.2).curveTo(-64.2,-63.6,-19.3,-105.6).curveTo(25.8,-147.5,86.9,-155.1).lineTo(101.1,-40.2).curveTo(77.3,-37.2,59.7,-20.9).curveTo(42.2,-4.5,37.6,19).curveTo(33,42.6,43.2,64.3).curveTo(53.3,86.1,74.3,97.7).lineTo(18.1,198.9).curveTo(-35.8,169.1,-61.8,113.1).closePath();
	var mask_graphics_144 = new cjs.Graphics().moveTo(-62.7,112.2).curveTo(-88.8,55.9,-76.3,-4.7).curveTo(-63.9,-65.2,-17.8,-107.1).curveTo(28.2,-148.9,89.7,-155.4).lineTo(101.7,-40.3).curveTo(77.8,-37.7,59.9,-21.5).curveTo(42,-5.2,37.1,18.4).curveTo(32.3,41.9,42.4,63.9).curveTo(52.6,85.9,73.7,97.7).lineTo(17.5,198.9).curveTo(-36.5,168.7,-62.7,112.2).closePath();
	var mask_graphics_145 = new cjs.Graphics().moveTo(-63.7,111.7).curveTo(-89.5,54.8,-76.5,-6.4).curveTo(-63.5,-67.5,-16.8,-108.9).curveTo(30,-150.3,92.3,-155.8).lineTo(102.4,-40.4).curveTo(78.1,-38.3,59.8,-22.1).curveTo(41.7,-6,36.7,17.7).curveTo(31.7,41.5,41.7,63.7).curveTo(51.8,85.9,73.1,97.7).lineTo(16.9,198.9).curveTo(-37.8,168.7,-63.7,111.7).closePath();
	var mask_graphics_146 = new cjs.Graphics().moveTo(-64.6,110.7).curveTo(-90.6,53.3,-76.9,-7.9).curveTo(-63.3,-69.1,-15.5,-110.4).curveTo(32.3,-151.6,94.9,-156.1).lineTo(103,-40.6).curveTo(78.7,-38.8,60,-22.8).curveTo(41.4,-6.7,36.1,17.1).curveTo(30.9,41,40.9,63.3).curveTo(51,85.7,72.4,97.7).lineTo(16.2,198.9).curveTo(-38.6,168.3,-64.6,110.7).closePath();
	var mask_graphics_147 = new cjs.Graphics().moveTo(-65.6,110.2).curveTo(-91.5,52.2,-77.3,-9.5).curveTo(-62.9,-71.3,-14.3,-112.2).curveTo(34.3,-153,97.7,-156.2).lineTo(103.6,-40.6).curveTo(79,-39.3,60,-23.5).curveTo(41.2,-7.5,35.7,16.5).curveTo(30.1,40.6,40.1,63).curveTo(50.1,85.7,71.8,97.7).lineTo(15.6,198.9).curveTo(-39.8,168.1,-65.6,110.2).closePath();
	var mask_graphics_148 = new cjs.Graphics().moveTo(-66.6,109.2).curveTo(-92.5,50.7,-77.6,-11.2).curveTo(-62.5,-73.1,-13,-113.7).curveTo(36.7,-154.2,100.3,-156.3).lineTo(104.3,-40.7).curveTo(79.5,-39.7,60.2,-24).curveTo(40.9,-8.2,35.1,15.8).curveTo(29.3,39.9,39.3,62.6).curveTo(49.5,85.5,71.1,97.7).lineTo(14.9,198.9).curveTo(-40.7,167.9,-66.6,109.2).closePath();
	var mask_graphics_149 = new cjs.Graphics().moveTo(-67.6,108.7).curveTo(-93.2,49.6,-77.6,-12.8).curveTo(-62,-75.2,-11.8,-115.3).curveTo(38.6,-155.3,103,-156.5).lineTo(104.9,-40.7).curveTo(80,-40.3,60.4,-24.7).curveTo(40.7,-9,34.6,15.2).curveTo(28.5,39.6,38.6,62.5).curveTo(48.6,85.5,70.5,97.7).lineTo(14.3,198.9).curveTo(-41.9,167.7,-67.6,108.7).closePath();
	var mask_graphics_150 = new cjs.Graphics().moveTo(-68.5,107.7).curveTo(-94.3,48.1,-78,-14.4).curveTo(-61.6,-76.9,-10.3,-116.7).curveTo(41,-156.5,105.6,-156.5).lineTo(105.6,-40.7).curveTo(80.4,-40.7,60.4,-25.3).curveTo(40.5,-9.7,34.1,14.6).curveTo(27.7,38.9,37.8,62.1).curveTo(47.8,85.4,69.9,97.7).lineTo(13.7,198.9).curveTo(-42.8,167.3,-68.5,107.7).closePath();
	var mask_graphics_151 = new cjs.Graphics().moveTo(-70.6,107.2).curveTo(-96,47.1,-79.2,-16).curveTo(-62.3,-79,-10.2,-118.3).curveTo(42,-157.6,107.2,-156.5).lineTo(105.3,-40.7).curveTo(79.9,-41.1,59.6,-25.8).curveTo(39.2,-10.5,32.6,14).curveTo(26.1,38.5,36,62).curveTo(46,85.4,68.2,97.7).lineTo(12,198.9).curveTo(-45,167.3,-70.6,107.2).closePath();
	var mask_graphics_152 = new cjs.Graphics().moveTo(-72.5,106.1).curveTo(-98.1,45.4,-80.5,-17.7).curveTo(-62.8,-80.6,-9.7,-119.6).curveTo(43.5,-158.6,108.9,-156.3).lineTo(104.9,-40.7).curveTo(79.4,-41.5,58.7,-26.3).curveTo(38.1,-11.2,31.2,13.3).curveTo(24.3,38,34.2,61.6).curveTo(44.2,85.3,66.5,97.7).lineTo(10.3,198.9).curveTo(-46.9,166.9,-72.5,106.1).closePath();
	var mask_graphics_153 = new cjs.Graphics().moveTo(-74.6,105.6).curveTo(-99.9,44.5,-81.6,-19.2).curveTo(-63.3,-82.8,-9.4,-121.3).curveTo(44.5,-159.7,110.5,-156.2).lineTo(104.6,-40.6).curveTo(78.7,-41.9,57.7,-27).curveTo(36.9,-12,29.7,12.8).curveTo(22.7,37.6,32.4,61.3).curveTo(42.3,85.1,64.9,97.7).lineTo(8.7,198.9).curveTo(-49.1,166.8,-74.6,105.6).closePath();
	var mask_graphics_154 = new cjs.Graphics().moveTo(-76.5,104.6).curveTo(-101.9,42.9,-82.9,-20.8).curveTo(-63.8,-84.4,-9,-122.6).curveTo(46,-160.7,112.2,-156.1).lineTo(104.1,-40.6).curveTo(78.4,-42.3,57,-27.6).curveTo(35.6,-12.7,28.1,12.1).curveTo(20.8,36.9,30.7,60.9).curveTo(40.6,85,63.2,97.7).lineTo(7,198.9).curveTo(-51.1,166.5,-76.5,104.6).closePath();
	var mask_graphics_155 = new cjs.Graphics().moveTo(-78.4,104.1).curveTo(-103.6,41.9,-84,-22.3).curveTo(-64.4,-86.5,-8.7,-124.1).curveTo(47,-161.6,113.9,-155.8).lineTo(103.8,-40.4).curveTo(77.7,-42.7,56,-28.1).curveTo(34.4,-13.5,26.7,11.4).curveTo(19.1,36.5,28.8,60.7).curveTo(38.7,85,61.6,97.7).lineTo(5.4,198.9).curveTo(-53.2,166.4,-78.4,104.1).closePath();
	var mask_graphics_156 = new cjs.Graphics().moveTo(-80.3,103.1).curveTo(-105.5,40.3,-85.1,-24).curveTo(-64.6,-88.2,-8,-125.5).curveTo(48.6,-162.6,115.5,-155.4).lineTo(103.4,-40.3).curveTo(77.4,-43,55.4,-28.6).curveTo(33.3,-14.2,25.3,10.9).curveTo(17.3,36,27.1,60.3).curveTo(36.9,84.8,60,97.7).lineTo(3.8,198.9).curveTo(-55.1,166,-80.3,103.1).closePath();
	var mask_graphics_157 = new cjs.Graphics().moveTo(-82.4,102.6).curveTo(-107.3,39.2,-86.4,-25.5).curveTo(-65.2,-90.3,-7.8,-126.8).curveTo(49.6,-163.4,117.2,-155.1).lineTo(103,-40.2).curveTo(76.7,-43.4,54.3,-29.2).curveTo(32,-15,23.7,10.2).curveTo(15.6,35.6,25.4,60.2).curveTo(35.1,84.8,58.3,97.7).lineTo(2.1,198.9).curveTo(-57.4,166,-82.4,102.6).closePath();
	var mask_graphics_158 = new cjs.Graphics().moveTo(-84.3,101.5).curveTo(-109.2,37.6,-87.4,-27.2).curveTo(-65.5,-91.9,-7.1,-128).curveTo(51.2,-164.2,118.8,-154.6).lineTo(102.7,-40).curveTo(76.4,-43.7,53.7,-29.6).curveTo(30.9,-15.5,22.4,9.7).curveTo(13.9,34.9,23.6,59.8).curveTo(33.4,84.7,56.6,97.7).lineTo(0.4,198.9).curveTo(-59.3,165.6,-84.3,101.5).closePath();
	var mask_graphics_159 = new cjs.Graphics().moveTo(-86.2,101).curveTo(-111,36.6,-88.5,-28.6).curveTo(-66,-93.9,-6.9,-129.4).curveTo(52.3,-164.9,120.4,-154.2).lineTo(102.3,-39.7).curveTo(75.7,-43.9,52.7,-30.1).curveTo(29.7,-16.3,20.9,9).curveTo(12.2,34.5,21.8,59.5).curveTo(31.5,84.6,55,97.7).lineTo(-1.2,198.9).curveTo(-61.4,165.4,-86.2,101).closePath();
	var mask_graphics_160 = new cjs.Graphics().moveTo(-88.1,100).curveTo(-112.9,35,-89.6,-30.3).curveTo(-66.3,-95.5,-6.2,-130.6).curveTo(53.9,-165.7,122.1,-153.6).lineTo(102,-39.6).curveTo(75.5,-44.3,52,-30.7).curveTo(28.6,-17,19.5,8.3).curveTo(10.5,33.8,20.1,59.1).curveTo(29.7,84.4,53.4,97.7).lineTo(-2.8,198.9).curveTo(-63.3,165,-88.1,100).closePath();
	var mask_graphics_161 = new cjs.Graphics().moveTo(-90,99.5).curveTo(-114.5,34.1,-90.7,-31.8).curveTo(-66.7,-97.4,-5.9,-132).curveTo(55,-166.4,123.7,-153).lineTo(101.6,-39.3).curveTo(74.9,-44.5,51.2,-31.1).curveTo(27.5,-17.7,18.2,7.9).curveTo(8.8,33.5,18.3,59).curveTo(27.9,84.4,51.8,97.7).lineTo(-4.4,198.9).curveTo(-65.5,165,-90,99.5).closePath();
	var mask_graphics_162 = new cjs.Graphics().moveTo(-92,98.8).curveTo(-116.4,33,-91.6,-33.1).curveTo(-66.9,-99.1,-5.1,-133.1).curveTo(56.8,-167,125.3,-152.3).lineTo(101.2,-39.1).curveTo(74.5,-44.8,50.4,-31.6).curveTo(26.4,-18.3,16.7,7.4).curveTo(7.1,33.1,16.6,58.7).curveTo(26.2,84.3,50.1,97.7).lineTo(-6.1,198.9).curveTo(-67.5,164.6,-92,98.8).closePath();
	var mask_graphics_163 = new cjs.Graphics().moveTo(-93.9,98).curveTo(-118.2,31.4,-92.7,-34.9).curveTo(-67.3,-101,-4.7,-134.3).curveTo(57.9,-167.6,126.9,-151.6).lineTo(100.9,-38.8).curveTo(74,-45,49.6,-32.2).curveTo(25.2,-19.2,15.3,6.6).curveTo(5.5,32.4,14.9,58.3).curveTo(24.4,84.3,48.5,97.7).lineTo(-7.7,198.9).curveTo(-69.7,164.6,-93.9,98).closePath();
	var mask_graphics_164 = new cjs.Graphics().moveTo(-95.8,97.3).curveTo(-120.1,30.4,-93.8,-36.1).curveTo(-67.5,-102.6,-4,-135.5).curveTo(59.6,-168.3,128.6,-150.8).lineTo(100.5,-38.5).curveTo(73.7,-45.3,48.9,-32.6).curveTo(24.1,-19.7,13.9,6.2).curveTo(3.7,32,13.2,58).curveTo(22.7,84.2,46.9,97.7).lineTo(-9.3,198.9).curveTo(-71.6,164.2,-95.8,97.3).closePath();
	var mask_graphics_165 = new cjs.Graphics().moveTo(-97.7,96.4).curveTo(-121.6,28.6,-94.7,-38).curveTo(-67.7,-104.5,-3.5,-136.6).curveTo(60.8,-168.7,130.1,-150).lineTo(100.2,-38.1).curveTo(73.2,-45.4,48.2,-33).curveTo(23.1,-20.5,12.6,5.4).curveTo(2.1,31.4,11.5,57.8).curveTo(20.8,84.2,45.3,97.7).lineTo(-10.9,198.9).curveTo(-73.7,164.1,-97.7,96.4).closePath();
	var mask_graphics_166 = new cjs.Graphics().moveTo(-99.6,95.7).curveTo(-123.5,27.7,-95.8,-39.2).curveTo(-67.9,-106.1,-2.7,-137.7).curveTo(62.5,-169.2,131.7,-149.2).lineTo(99.8,-37.8).curveTo(72.8,-45.6,47.4,-33.4).curveTo(22,-21.1,11.2,4.9).curveTo(0.5,30.9,9.7,57.4).curveTo(19.1,83.9,43.7,97.7).lineTo(-12.5,198.9).curveTo(-75.6,163.7,-99.6,95.7).closePath();
	var mask_graphics_167 = new cjs.Graphics().moveTo(-101.4,94.9).curveTo(-125.1,26.1,-96.7,-41).curveTo(-68.2,-108,-2.3,-138.7).curveTo(63.7,-169.5,133.3,-148.2).lineTo(99.4,-37.4).curveTo(72.4,-45.7,46.6,-33.8).curveTo(21,-21.9,9.9,4.3).curveTo(-1,30.4,8.2,57.1).curveTo(17.4,83.9,42.2,97.7).lineTo(-14,198.9).curveTo(-77.7,163.7,-101.4,94.9).closePath();
	var mask_graphics_168 = new cjs.Graphics().moveTo(-103.3,94.1).curveTo(-126.8,25,-97.6,-42.3).curveTo(-68.2,-109.5,-1.3,-139.7).curveTo(65.6,-169.9,134.9,-147.1).lineTo(99,-37).curveTo(72.1,-46,46.1,-34.2).curveTo(20.1,-22.4,8.6,3.7).curveTo(-2.8,30,6.4,56.8).curveTo(15.6,83.8,40.5,97.7).lineTo(-15.7,198.9).curveTo(-79.6,163.3,-103.3,94.1).closePath();
	var mask_graphics_169 = new cjs.Graphics().moveTo(-105.1,93.2).curveTo(-128.4,23.4,-98.5,-44.1).curveTo(-68.5,-111.4,-1,-140.8).curveTo(66.7,-170.2,136.5,-146.2).lineTo(98.7,-36.6).curveTo(71.6,-46,45.2,-34.6).curveTo(18.9,-23.1,7.3,3).curveTo(-4.4,29.3,4.7,56.5).curveTo(13.8,83.8,39,97.7).lineTo(-17.2,198.9).curveTo(-81.7,163.3,-105.1,93.2).closePath();
	var mask_graphics_170 = new cjs.Graphics().moveTo(-106.9,92.6).curveTo(-130.2,22.3,-99.4,-45.3).curveTo(-68.5,-112.9,0,-141.7).curveTo(68.6,-170.6,138,-145.1).lineTo(98.4,-36.2).curveTo(71.4,-46.1,44.7,-35).curveTo(18.1,-23.8,6,2.5).curveTo(-6.1,28.9,3,56.3).curveTo(12.2,83.6,37.4,97.7).lineTo(-18.8,198.9).curveTo(-83.5,162.8,-106.9,92.6).closePath();
	var mask_graphics_171 = new cjs.Graphics().moveTo(-108.8,91.6).curveTo(-131.8,20.7,-100.2,-47.1).curveTo(-68.7,-114.6,0.5,-142.7).curveTo(69.9,-170.7,139.6,-143.9).lineTo(98,-35.8).curveTo(70.9,-46.2,44,-35.4).curveTo(17,-24.4,4.7,1.8).curveTo(-7.6,28.2,1.3,55.9).curveTo(10.4,83.6,35.9,97.7).lineTo(-20.3,198.9).curveTo(-85.6,162.7,-108.8,91.6).closePath();
	var mask_graphics_172 = new cjs.Graphics().moveTo(-110.4,90.9).curveTo(-133.4,19.7,-101.1,-48.3).curveTo(-68.5,-116.3,1.2,-143.6).curveTo(71.1,-171,141.1,-142.7).lineTo(97.8,-35.3).curveTo(70.5,-46.4,43.3,-35.7).curveTo(16.2,-25,3.5,1.4).curveTo(-9.1,27.8,-0.2,55.6).curveTo(8.8,83.4,34.4,97.7).lineTo(-21.8,198.9).curveTo(-87.4,162.3,-110.4,90.9).closePath();
	var mask_graphics_173 = new cjs.Graphics().moveTo(-112.2,90.1).curveTo(-135,17.9,-101.8,-50).curveTo(-68.6,-118,2.2,-144.6).curveTo(73,-171,142.6,-141.4).lineTo(97.4,-34.9).curveTo(70.3,-46.4,42.7,-36.1).curveTo(15.1,-25.7,2.2,0.7).curveTo(-10.7,27.3,-1.9,55.3).curveTo(7.1,83.4,32.8,97.7).lineTo(-23.4,198.9).curveTo(-89.5,162.3,-112.2,90.1).closePath();
	var mask_graphics_174 = new cjs.Graphics().moveTo(-114.1,89.4).curveTo(-136.7,17,-102.6,-51.3).curveTo(-68.5,-119.5,2.9,-145.4).curveTo(74.4,-171.1,144.1,-140.1).lineTo(97.1,-34.3).curveTo(69.9,-46.4,42,-36.4).curveTo(14.3,-26.3,1,0.2).curveTo(-12.3,26.9,-3.5,55.1).curveTo(5.5,83.2,31.3,97.7).lineTo(-24.9,198.9).curveTo(-91.4,161.9,-114.1,89.4).closePath();
	var mask_graphics_175 = new cjs.Graphics().moveTo(-115.8,88.5).curveTo(-138.2,15.2,-103.5,-53).curveTo(-68.7,-121.3,3.8,-146.2).curveTo(76.2,-171.1,145.7,-138.7).lineTo(96.7,-33.8).curveTo(69.7,-46.4,41.4,-36.8).curveTo(13.2,-27,-0.3,-0.5).curveTo(-13.8,26.2,-5.2,54.6).curveTo(3.6,83.2,29.8,97.7).lineTo(-26.4,198.9).curveTo(-93.3,161.8,-115.8,88.5).closePath();
	var mask_graphics_176 = new cjs.Graphics().moveTo(-117.6,87.8).curveTo(-139.8,14.3,-104.2,-54.2).curveTo(-68.4,-122.6,4.6,-147).curveTo(77.6,-171.2,147.2,-137.3).lineTo(96.4,-33.2).curveTo(69.3,-46.4,40.9,-37).curveTo(12.4,-27.6,-1.5,-0.9).curveTo(-15.3,25.8,-6.7,54.4).curveTo(2,83.1,28.3,97.7).lineTo(-27.9,198.9).curveTo(-95.2,161.4,-117.6,87.8).closePath();
	var mask_graphics_177 = new cjs.Graphics().moveTo(-119.3,86.9).curveTo(-141.3,12.5,-104.8,-56).curveTo(-68.4,-124.4,5.5,-147.7).curveTo(79.5,-171,148.7,-135.8).lineTo(96,-32.7).curveTo(69.2,-46.4,40.3,-37.3).curveTo(11.5,-28.2,-2.7,-1.6).curveTo(-16.8,25.1,-8.3,54.1).curveTo(0.4,83.1,26.8,97.7).lineTo(-29.4,198.9).curveTo(-97.3,161.4,-119.3,86.9).closePath();
	var mask_graphics_178 = new cjs.Graphics().moveTo(-120.9,86.2).curveTo(-142.8,11.6,-105.5,-57.2).curveTo(-68.1,-125.9,6.4,-148.5).curveTo(80.9,-171,150.1,-134.3).lineTo(95.8,-32).curveTo(68.8,-46.4,39.9,-37.6).curveTo(10.9,-28.8,-3.7,-2.1).curveTo(-18.2,24.7,-9.7,53.7).curveTo(-1.2,82.8,25.4,97.7).lineTo(-30.8,198.9).curveTo(-99,161,-120.9,86.2).closePath();
	var mask_graphics_179 = new cjs.Graphics().moveTo(-122.7,85.3).curveTo(-144.3,9.8,-106.3,-58.8).curveTo(-68.1,-127.5,7.4,-149.2).curveTo(82.9,-170.8,151.6,-132.6).lineTo(95.4,-31.5).curveTo(68.7,-46.2,39.3,-37.8).curveTo(9.9,-29.5,-5,-2.8).curveTo(-19.7,24,-11.3,53.4).curveTo(-2.9,82.8,23.9,97.7).lineTo(-32.3,198.9).curveTo(-101,160.8,-122.7,85.3).closePath();
	var mask_graphics_180 = new cjs.Graphics().moveTo(-113.9,112.2).curveTo(-140.1,55.9,-127.9,-4.4).curveTo(-115.5,-64.7,-69.8,-106.6).curveTo(-23.9,-148.6,37.5,-155.4).curveTo(99,-162.2,153,-131).lineTo(95.2,-30.8).curveTo(74.1,-42.9,50.1,-40.3).curveTo(26.2,-37.6,8.4,-21.3).curveTo(-9.4,-4.9,-14.2,18.5).curveTo(-19,41.9,-8.8,63.9).curveTo(1.3,85.9,22.5,97.7).lineTo(-33.7,198.9).curveTo(-87.8,168.7,-113.9,112.2).closePath();
	var mask_graphics_181 = new cjs.Graphics().moveTo(-115.5,111.8).curveTo(-141.6,54.9,-128.9,-5.6).curveTo(-116.2,-66.2,-69.6,-108).curveTo(-22.8,-149.7,39,-155.7).curveTo(100.9,-161.5,154.4,-129.4).lineTo(94.8,-30.1).curveTo(74,-42.6,49.9,-40.3).curveTo(25.8,-38,7.6,-21.7).curveTo(-10.5,-5.5,-15.5,18.1).curveTo(-20.4,41.6,-10.2,63.7).curveTo(-0.1,85.9,21,97.7).lineTo(-35.2,198.9).curveTo(-89.3,168.7,-115.5,111.8).closePath();
	var mask_graphics_182 = new cjs.Graphics().moveTo(-117.4,111.1).curveTo(-143.3,54.1,-130.2,-6.8).curveTo(-116.9,-67.8,-69.5,-109.4).curveTo(-22.1,-150.9,40.1,-156.1).curveTo(102.2,-161.1,155.8,-127.6).lineTo(94.5,-29.5).curveTo(73.6,-42.5,49.4,-40.6).curveTo(25.2,-38.5,6.7,-22.4).curveTo(-11.7,-6.2,-16.8,17.5).curveTo(-22,41.2,-11.9,63.4).curveTo(-1.8,85.8,19.6,97.7).lineTo(-36.6,198.9).curveTo(-91.4,168.3,-117.4,111.1).closePath();
	var mask_graphics_183 = new cjs.Graphics().moveTo(-118.9,110.7).curveTo(-144.8,53.2,-131.1,-8.1).curveTo(-117.3,-69.3,-69.2,-110.6).curveTo(-21,-151.9,41.6,-156.1).curveTo(104.1,-160.3,157.2,-125.9).lineTo(94.2,-28.8).curveTo(73.5,-42.2,49.1,-40.6).curveTo(24.8,-38.9,5.9,-22.8).curveTo(-12.8,-6.7,-18.2,17.1).curveTo(-23.5,41,-13.3,63.3).curveTo(-3.1,85.7,18.3,97.7).lineTo(-37.9,198.9).curveTo(-92.9,168.3,-118.9,110.7).closePath();
	var mask_graphics_184 = new cjs.Graphics().moveTo(-120.6,110.2).curveTo(-146.5,52.2,-132.3,-9.5).curveTo(-117.9,-71.3,-69.3,-112.2).curveTo(-20.7,-153,42.7,-156.3).curveTo(106.1,-159.6,158.6,-124.1).lineTo(93.9,-28.1).curveTo(73.4,-41.9,48.7,-40.7).curveTo(24,-39.3,5.1,-23.5).curveTo(-13.8,-7.5,-19.3,16.5).curveTo(-24.9,40.6,-14.9,63).curveTo(-4.8,85.7,16.8,97.7).lineTo(-39.4,198.9).curveTo(-94.8,168.1,-120.6,110.2).closePath();
	var mask_graphics_185 = new cjs.Graphics().moveTo(-122.2,109.2).curveTo(-148.1,50.7,-133.4,-10.9).curveTo(-118.6,-72.4,-69.2,-113.1).curveTo(-19.7,-153.8,43.8,-156.3).curveTo(107.4,-158.8,160,-122.2).lineTo(93.6,-27.3).curveTo(73,-41.6,48.3,-40.7).curveTo(23.6,-39.6,4.4,-23.8).curveTo(-14.9,-7.9,-20.7,16).curveTo(-26.4,40,-16.3,62.8).curveTo(-6.2,85.5,15.5,97.7).lineTo(-40.7,198.9).curveTo(-96.4,167.9,-122.2,109.2).closePath();
	var mask_graphics_186 = new cjs.Graphics().moveTo(-123.7,108.7).curveTo(-149.6,49.8,-134.3,-12.1).curveTo(-119,-73.9,-68.9,-114.4).curveTo(-18.6,-154.7,45.3,-156.3).curveTo(109.3,-157.8,161.3,-120.3).lineTo(93.2,-26.6).curveTo(73,-41.2,48.1,-40.7).curveTo(23.2,-40,3.7,-24.3).curveTo(-15.8,-8.6,-21.8,15.5).curveTo(-27.7,39.6,-17.7,62.5).curveTo(-7.7,85.5,14.1,97.7).lineTo(-42.1,198.9).curveTo(-97.9,167.7,-123.7,108.7).closePath();
	var mask_graphics_187 = new cjs.Graphics().moveTo(-125.6,108.1).curveTo(-151.2,49,-135.4,-13.3).curveTo(-119.5,-75.5,-68.6,-115.7).curveTo(-17.7,-155.9,46.5,-156.6).curveTo(110.7,-157.3,162.7,-118.3).lineTo(92.9,-25.8).curveTo(72.8,-41,47.7,-40.7).curveTo(22.8,-40.4,2.9,-24.8).curveTo(-16.9,-9.1,-23.1,15).curveTo(-29.2,39.2,-19.3,62.2).curveTo(-9.3,85.4,12.8,97.7).lineTo(-43.4,198.9).curveTo(-99.9,167.3,-125.6,108.1).closePath();
	var mask_graphics_188 = new cjs.Graphics().moveTo(-127.1,107.6).curveTo(-152.6,48,-136.4,-14.6).curveTo(-120,-77,-68.3,-116.9).curveTo(-16.5,-156.8,48.1,-156.5).curveTo(112.6,-156.2,164,-116.3).lineTo(92.7,-25).curveTo(72.7,-40.6,47.5,-40.7).curveTo(22.5,-40.8,2.3,-25.4).curveTo(-17.9,-9.8,-24.3,14.6).curveTo(-30.6,38.9,-20.6,62.1).curveTo(-10.6,85.4,11.5,97.7).lineTo(-44.7,198.9).curveTo(-101.3,167.3,-127.1,107.6).closePath();
	var mask_graphics_189 = new cjs.Graphics().moveTo(-128.6,107.2).curveTo(-154.1,47.1,-137.3,-16).curveTo(-120.3,-79,-68.2,-118.3).curveTo(-16.1,-157.6,49.2,-156.5).curveTo(114.5,-155.3,165.3,-114.2).lineTo(92.4,-24.3).curveTo(72.6,-40.3,47.2,-40.7).curveTo(21.8,-41.1,1.5,-25.8).curveTo(-18.8,-10.5,-25.4,14).curveTo(-31.9,38.5,-22,62).curveTo(-12,85.4,10.2,97.7).lineTo(-46,198.9).curveTo(-103,167.3,-128.6,107.2).closePath();
	var mask_graphics_190 = new cjs.Graphics().moveTo(-130.2,106.2).curveTo(-155.6,45.6,-138.3,-17.3).curveTo(-120.8,-80.1,-67.9,-119.2).curveTo(-14.9,-158.4,50.5,-156.3).curveTo(115.9,-154.3,166.5,-112.1).lineTo(92.1,-23.4).curveTo(72.4,-39.9,47,-40.7).curveTo(21.5,-41.5,0.8,-26.2).curveTo(-19.8,-10.9,-26.6,13.5).curveTo(-33.3,38,-23.5,61.6).curveTo(-13.4,85.3,8.9,97.7).lineTo(-47.3,198.9).curveTo(-104.6,166.9,-130.2,106.2).closePath();
	var mask_graphics_191 = new cjs.Graphics().moveTo(-131.5,105.7).curveTo(-157,44.6,-139.1,-18.5).curveTo(-121.1,-81.6,-67.5,-120.5).curveTo(-13.7,-159.2,52.1,-156.2).curveTo(117.9,-153.1,167.8,-110).lineTo(91.9,-22.5).curveTo(72.4,-39.3,46.8,-40.6).curveTo(21.2,-41.8,0.2,-26.7).curveTo(-20.6,-11.6,-27.7,12.9).curveTo(-34.6,37.6,-24.7,61.3).curveTo(-14.7,85.1,7.7,97.7).lineTo(-48.5,198.9).curveTo(-105.9,166.9,-131.5,105.7).closePath();
	var mask_graphics_192 = new cjs.Graphics().moveTo(-133.2,105).curveTo(-158.5,43.7,-140.1,-19.7).curveTo(-121.6,-83.1,-67.1,-121.7).curveTo(-12.6,-160.1,53.3,-156.2).curveTo(119.2,-152.3,169.1,-107.7).lineTo(91.6,-21.7).curveTo(72.2,-39.1,46.5,-40.7).curveTo(20.9,-42.2,-0.4,-27.2).curveTo(-21.6,-12.1,-28.8,12.5).curveTo(-36,37.2,-26.1,61).curveTo(-16.2,85,6.4,97.7).lineTo(-49.8,198.9).curveTo(-107.9,166.5,-133.2,105).closePath();
	var mask_graphics_193 = new cjs.Graphics().moveTo(-134.6,104.6).curveTo(-159.9,42.9,-140.8,-20.9).curveTo(-121.7,-84.6,-66.6,-122.8).curveTo(-11.3,-160.9,54.9,-156.1).curveTo(121.2,-151.1,170.3,-105.6).lineTo(91.3,-20.9).curveTo(72.2,-38.5,46.4,-40.6).curveTo(20.6,-42.5,-0.9,-27.6).curveTo(-22.4,-12.7,-29.9,12.1).curveTo(-37.2,36.9,-27.4,60.9).curveTo(-17.6,85,5.2,97.7).lineTo(-51,198.9).curveTo(-109.2,166.4,-134.6,104.6).closePath();
	var mask_graphics_194 = new cjs.Graphics().moveTo(-136.1,104.1).curveTo(-161.2,41.9,-141.6,-22.3).curveTo(-122,-86.5,-66.3,-124.1).curveTo(-10.7,-161.6,56.2,-155.8).curveTo(123.1,-149.8,171.5,-103.3).lineTo(91,-20).curveTo(72.2,-38.1,46.1,-40.4).curveTo(20.1,-42.7,-1.6,-28.1).curveTo(-23.3,-13.5,-31,11.4).curveTo(-38.6,36.5,-28.8,60.7).curveTo(-18.9,85,4,97.7).lineTo(-52.2,198.9).curveTo(-110.9,166.4,-136.1,104.1).closePath();
	var mask_graphics_195 = new cjs.Graphics().moveTo(-137.5,103.1).curveTo(-162.7,40.3,-142.5,-23.6).curveTo(-122.3,-87.5,-65.8,-124.9).curveTo(-9.4,-162.3,57.5,-155.5).curveTo(124.6,-148.8,172.6,-101).lineTo(90.7,-19).curveTo(72,-37.7,45.9,-40.4).curveTo(19.9,-43,-2.1,-28.5).curveTo(-24,-13.9,-31.8,11).curveTo(-39.7,36,-30,60.3).curveTo(-20.2,84.8,2.8,97.7).lineTo(-53.4,198.9).curveTo(-112.3,166,-137.5,103.1).closePath();
	var mask_graphics_196 = new cjs.Graphics().moveTo(-138.8,102.6).curveTo(-163.8,39.3,-143.1,-24.8).curveTo(-122.4,-89,-65.2,-126).curveTo(-7.9,-163,59.2,-155.1).curveTo(126.5,-147.3,173.8,-98.7).lineTo(90.5,-18.2).curveTo(72.1,-37,46,-40.2).curveTo(19.8,-43.3,-2.5,-28.9).curveTo(-24.7,-14.4,-32.9,10.5).curveTo(-41,35.6,-31.2,60.2).curveTo(-21.4,84.8,1.7,97.7).lineTo(-54.5,198.9).curveTo(-113.6,166,-138.8,102.6).closePath();
	var mask_graphics_197 = new cjs.Graphics().moveTo(-140.5,102).curveTo(-165.4,38.5,-144.1,-26.1).curveTo(-122.7,-90.5,-64.8,-127.2).curveTo(-6.7,-163.8,60.6,-155).curveTo(128,-146.2,174.9,-96.2).lineTo(90.3,-17.3).curveTo(72,-36.8,45.7,-40.2).curveTo(19.6,-43.5,-3,-29.3).curveTo(-25.6,-15,-34,10.1).curveTo(-42.3,35.1,-32.5,59.9).curveTo(-22.8,84.7,0.5,97.7).lineTo(-55.7,198.9).curveTo(-115.4,165.6,-140.5,102).closePath();
	var mask_graphics_198 = new cjs.Graphics().moveTo(-141.5,101.5).curveTo(-166.5,37.6,-144.7,-27.2).curveTo(-122.7,-91.9,-63.9,-128.2).curveTo(-5.2,-164.3,62.4,-154.6).curveTo(130,-144.7,176,-93.8).lineTo(90,-16.3).curveTo(72,-36.1,45.7,-40).curveTo(19.5,-43.8,-3.4,-29.7).curveTo(-26.3,-15.6,-34.8,9.5).curveTo(-43.4,34.9,-33.6,59.8).curveTo(-23.9,84.7,-0.6,97.7).lineTo(-56.8,198.9).curveTo(-116.6,165.6,-141.5,101.5).closePath();
	var mask_graphics_199 = new cjs.Graphics().moveTo(-143,101).curveTo(-167.7,36.6,-145.3,-28.6).curveTo(-122.8,-93.9,-63.6,-129.4).curveTo(-4.4,-164.9,63.7,-154.2).curveTo(131.8,-143.3,177.2,-91.2).lineTo(89.7,-15.4).curveTo(72.1,-35.5,45.5,-39.7).curveTo(19,-43.9,-4,-30.1).curveTo(-27,-16.3,-35.8,9).curveTo(-44.5,34.5,-34.9,59.5).curveTo(-25.3,84.6,-1.7,97.7).lineTo(-57.9,198.9).curveTo(-118.2,165.4,-143,101).closePath();
	var mask_graphics_200 = new cjs.Graphics().moveTo(-144.3,100).curveTo(-169,35,-146,-30).curveTo(-122.9,-94.9,-63,-130.2).curveTo(-3,-165.6,65.1,-153.8).curveTo(133.4,-141.9,178.2,-88.8).lineTo(89.5,-14.3).curveTo(72.2,-35,45.6,-39.6).curveTo(19.1,-44.2,-4.4,-30.5).curveTo(-27.7,-16.7,-36.7,8.6).curveTo(-45.7,33.9,-36,59.1).curveTo(-26.3,84.4,-2.7,97.7).lineTo(-58.9,198.9).curveTo(-119.5,165.2,-144.3,100).closePath();
	var mask_graphics_201 = new cjs.Graphics().moveTo(-145.5,99.5).curveTo(-170.1,34.1,-146.6,-31.1).curveTo(-122.9,-96.2,-62.2,-131.2).curveTo(-1.5,-166.1,66.9,-153.2).curveTo(135.3,-140.2,179.3,-86.2).lineTo(89.4,-13.3).curveTo(72.2,-34.3,45.5,-39.5).curveTo(18.9,-44.5,-4.8,-30.9).curveTo(-28.3,-17.3,-37.5,8.1).curveTo(-46.7,33.5,-37.1,59).curveTo(-27.5,84.4,-3.8,97.7).lineTo(-60,198.9).curveTo(-120.7,165,-145.5,99.5).closePath();
	var mask_graphics_202 = new cjs.Graphics().moveTo(-146.9,98.9).curveTo(-171.4,33.2,-147.2,-32.3).curveTo(-122.9,-97.8,-61.4,-132.4).curveTo(0,-166.8,68.4,-153).curveTo(136.8,-139,180.3,-83.6).lineTo(89,-12.4).curveTo(72.1,-33.9,45.4,-39.3).curveTo(18.9,-44.8,-5.1,-31.3).curveTo(-28.9,-17.8,-38.4,7.7).curveTo(-47.9,33.1,-38.4,58.7).curveTo(-28.8,84.3,-4.8,97.7).lineTo(-61,198.9).curveTo(-122.4,164.6,-146.9,98.9).closePath();
	var mask_graphics_203 = new cjs.Graphics().moveTo(-148,98.8).curveTo(-172.4,33,-147.6,-33.1).curveTo(-122.9,-99.2,-60.7,-133.2).curveTo(1.6,-167.2,70.1,-152.1).curveTo(138.8,-137.1,181.3,-81).lineTo(88.8,-11.3).curveTo(72.3,-33.1,45.6,-39.1).curveTo(18.9,-44.9,-5.3,-31.6).curveTo(-29.5,-18.3,-39.3,7.2).curveTo(-48.9,33,-39.4,58.6).curveTo(-29.8,84.3,-5.8,97.7).lineTo(-62,198.9).curveTo(-123.5,164.6,-148,98.8).closePath();
	var mask_graphics_204 = new cjs.Graphics().moveTo(-149.3,98).curveTo(-173.6,31.4,-148.1,-34.9).curveTo(-122.7,-101,-60.1,-134.3).curveTo(2.5,-167.6,71.5,-151.6).curveTo(140.6,-135.6,182.3,-78.3).lineTo(88.6,-10.2).curveTo(72.4,-32.6,45.4,-38.8).curveTo(18.6,-45,-5.8,-32.2).curveTo(-30.2,-19.2,-40,6.6).curveTo(-49.9,32.4,-40.4,58.3).curveTo(-31,84.3,-6.9,97.7).lineTo(-63.1,198.9).curveTo(-125.1,164.6,-149.3,98).closePath();
	var mask_graphics_205 = new cjs.Graphics().moveTo(-150.5,97.3).curveTo(-174.8,30.4,-148.8,-35.8).curveTo(-122.7,-102,-59.3,-135.1).curveTo(4.1,-168.1,73,-151.1).curveTo(142.1,-134,183.3,-75.6).lineTo(88.3,-9.3).curveTo(72.4,-31.9,45.4,-38.5).curveTo(18.6,-45.2,-6.1,-32.3).curveTo(-30.7,-19.4,-40.9,6.3).curveTo(-51,32,-41.5,58).curveTo(-32.1,84.2,-7.8,97.7).lineTo(-64,198.9).curveTo(-126.3,164.2,-150.5,97.3).closePath();
	var mask_graphics_206 = new cjs.Graphics().moveTo(-151.5,96.8).curveTo(-175.7,29.5,-149.2,-37).curveTo(-122.5,-103.4,-58.5,-135.9).curveTo(5.7,-168.4,74.8,-150.3).curveTo(144,-132,184.2,-72.9).lineTo(88.2,-8.2).curveTo(72.5,-31.2,45.5,-38.3).curveTo(18.7,-45.3,-6.3,-32.7).curveTo(-31.2,-20,-41.7,5.9).curveTo(-52,31.8,-42.5,57.9).curveTo(-33,84.2,-8.8,97.7).lineTo(-65,198.9).curveTo(-127.3,164.1,-151.5,96.8).closePath();
	var mask_graphics_207 = new cjs.Graphics().moveTo(-152.9,96.1).curveTo(-176.9,28.5,-149.7,-38.3).curveTo(-122.3,-104.9,-57.4,-137).curveTo(7.4,-168.9,76.5,-149.8).curveTo(145.6,-130.6,185.1,-70.1).lineTo(88,-7.1).curveTo(72.6,-30.7,45.6,-38.1).curveTo(18.8,-45.6,-6.5,-33.1).curveTo(-31.7,-20.6,-42.3,5.4).curveTo(-52.8,31.4,-43.5,57.6).curveTo(-34.2,84,-9.6,97.7).lineTo(-65.8,198.9).curveTo(-128.8,163.8,-152.9,96.1).closePath();
	var mask_graphics_208 = new cjs.Graphics().moveTo(-153.8,95.5).curveTo(-177.8,27.6,-150,-39.3).curveTo(-122.1,-106.2,-56.6,-137.8).curveTo(9.1,-169.2,78.3,-148.9).curveTo(147.5,-128.6,186,-67.4).lineTo(87.8,-6).curveTo(72.8,-29.9,45.8,-37.8).curveTo(19,-45.7,-6.6,-33.4).curveTo(-32.1,-21.1,-42.9,4.9).curveTo(-53.7,30.9,-44.5,57.4).curveTo(-35.2,83.9,-10.5,97.7).lineTo(-66.7,198.9).curveTo(-129.8,163.7,-153.8,95.5).closePath();
	var mask_graphics_209 = new cjs.Graphics().moveTo(-155,94.9).curveTo(-178.6,26.1,-150.2,-41).curveTo(-121.8,-108,-55.8,-138.7).curveTo(10.1,-169.5,79.7,-148.2).curveTo(149.3,-127,186.9,-64.5).lineTo(87.6,-4.9).curveTo(73,-29.2,45.9,-37.4).curveTo(18.8,-45.7,-6.9,-33.8).curveTo(-32.5,-21.9,-43.6,4.3).curveTo(-54.6,30.4,-45.4,57.1).curveTo(-36.2,83.9,-11.4,97.7).lineTo(-67.6,198.9).curveTo(-131.3,163.7,-155,94.9).closePath();
	var mask_graphics_210 = new cjs.Graphics().moveTo(-156,94.2).curveTo(-179.6,25.1,-150.6,-41.9).curveTo(-121.5,-108.9,-54.7,-139.4).curveTo(12,-169.9,81.5,-147.4).curveTo(151,-124.9,187.7,-61.7).lineTo(87.5,-3.7).curveTo(73.1,-28.4,46,-37.2).curveTo(19.1,-46,-6.9,-34.1).curveTo(-32.9,-22.1,-44.3,3.9).curveTo(-55.5,30,-46.3,56.8).curveTo(-37.1,83.8,-12.2,97.7).lineTo(-68.4,198.9).curveTo(-132.3,163.3,-156,94.2).closePath();
	var mask_graphics_211 = new cjs.Graphics().moveTo(-156.8,93.6).curveTo(-180.4,24.2,-150.8,-43.1).curveTo(-121.1,-110.3,-53.7,-140.2).curveTo(13.8,-170,83.3,-146.5).curveTo(152.9,-122.8,188.5,-58.8).lineTo(87.3,-2.6).curveTo(73.4,-27.6,46.3,-36.8).curveTo(19.2,-46,-7.1,-34.3).curveTo(-33.2,-22.7,-44.9,3.5).curveTo(-56.4,29.6,-47.2,56.7).curveTo(-37.9,83.8,-13,97.7).lineTo(-69.2,198.9).curveTo(-133.1,163.3,-156.8,93.6).closePath();
	var mask_graphics_212 = new cjs.Graphics().moveTo(-158.2,93).curveTo(-181.5,23.2,-151.2,-44.2).curveTo(-120.8,-111.7,-52.7,-141).curveTo(15.5,-170.4,84.9,-145.9).curveTo(154.4,-121.3,189.3,-55.9).lineTo(87.1,-1.6).curveTo(73.5,-27,46.4,-36.6).curveTo(19.5,-46.1,-7.1,-34.7).curveTo(-33.6,-23.2,-45.5,3).curveTo(-57.3,29.3,-48.2,56.4).curveTo(-39,83.6,-13.8,97.7).lineTo(-70,198.9).curveTo(-134.8,162.8,-158.2,93).closePath();
	var mask_graphics_213 = new cjs.Graphics().moveTo(-158.9,92.4).curveTo(-182.2,22.3,-151.4,-45.4).curveTo(-120.3,-113,-51.6,-141.9).curveTo(17.4,-170.6,86.7,-144.8).curveTo(156.2,-119,190,-53).lineTo(87,-0.3).curveTo(73.7,-26.1,46.6,-36.2).curveTo(19.7,-46.2,-7.1,-35).curveTo(-33.9,-23.8,-46,2.5).curveTo(-58.1,28.9,-49,56.3).curveTo(-39.9,83.6,-14.6,97.7).lineTo(-70.8,198.9).curveTo(-135.5,162.7,-158.9,92.4).closePath();
	var mask_graphics_214 = new cjs.Graphics().moveTo(-160,91.6).curveTo(-183,20.7,-151.4,-47.1).curveTo(-119.9,-114.6,-50.7,-142.7).curveTo(18.7,-170.7,88.4,-143.9).curveTo(158.2,-117.1,190.8,-50).lineTo(86.8,0.7).curveTo(74.1,-25.4,46.8,-35.8).curveTo(19.7,-46.2,-7.2,-35.4).curveTo(-34.2,-24.4,-46.5,1.8).curveTo(-58.8,28.2,-49.9,55.9).curveTo(-40.8,83.6,-15.3,97.7).lineTo(-71.5,198.9).curveTo(-136.8,162.7,-160,91.6).closePath();
	var mask_graphics_215 = new cjs.Graphics().moveTo(-160.8,90.9).curveTo(-183.9,19.7,-151.8,-48).curveTo(-119.5,-115.7,-49.9,-143.3).curveTo(19.8,-171,89.7,-143.1).curveTo(159.6,-115,191.5,-47.1).lineTo(86.6,2).curveTo(74.1,-24.6,46.9,-35.5).curveTo(19.8,-46.4,-7.4,-35.7).curveTo(-34.5,-24.8,-47.1,1.6).curveTo(-59.5,28,-50.6,55.6).curveTo(-41.7,83.4,-16.1,97.7).lineTo(-72.3,198.9).curveTo(-137.8,162.3,-160.8,90.9).closePath();
	var mask_graphics_216 = new cjs.Graphics().moveTo(-161.7,90.5).curveTo(-184.5,18.8,-151.8,-49.1).curveTo(-118.9,-116.9,-48.6,-144).curveTo(21.8,-171,91.6,-141.9).curveTo(161.5,-112.6,192.2,-43.9).lineTo(86.4,3).curveTo(74.5,-23.6,47.3,-35).curveTo(20.1,-46.4,-7.3,-35.8).curveTo(-34.6,-25.3,-47.5,1.2).curveTo(-60.2,27.6,-51.3,55.5).curveTo(-42.3,83.4,-16.7,97.7).lineTo(-73,198.9).curveTo(-138.6,162.3,-161.7,90.5).closePath();
	var mask_graphics_217 = new cjs.Graphics().moveTo(-162.7,89.9).curveTo(-185.5,17.8,-152,-50.3).curveTo(-118.5,-118.3,-47.4,-144.8).curveTo(23.7,-171.2,93.3,-141).curveTo(163,-110.8,192.9,-41).lineTo(86.3,4.3).curveTo(74.7,-23,47.6,-34.7).curveTo(20.5,-46.4,-7.3,-36.1).curveTo(-34.9,-25.8,-48,0.6).curveTo(-61,27.2,-52.2,55.2).curveTo(-43.3,83.2,-17.4,97.7).lineTo(-73.6,198.9).curveTo(-140,161.9,-162.7,89.9).closePath();
	var mask_graphics_218 = new cjs.Graphics().moveTo(-164.2,88.5).curveTo(-186.6,15.2,-151.9,-53).curveTo(-117.1,-121.3,-44.6,-146.2).curveTo(27.8,-171.1,97.1,-138.7).curveTo(166.6,-106.4,194.1,-34.9).lineTo(86,6.7).curveTo(75.3,-21.2,48.3,-33.8).curveTo(21.3,-46.4,-7,-36.8).curveTo(-35.2,-27,-48.7,-0.5).curveTo(-62.3,26.2,-53.6,54.6).curveTo(-44.8,83.2,-18.6,97.7).lineTo(-74.8,198.9).curveTo(-141.7,161.8,-164.2,88.5).closePath();
	var mask_graphics_219 = new cjs.Graphics().moveTo(-165.1,87.8).curveTo(-187.3,14.3,-152,-54).curveTo(-116.5,-122.2,-43.6,-146.7).curveTo(29.2,-171.2,98.7,-137.7).curveTo(168.2,-103.9,194.7,-31.8).lineTo(85.8,7.9).curveTo(75.5,-20.2,48.5,-33.4).curveTo(21.4,-46.4,-6.9,-36.9).curveTo(-35.2,-27.3,-49,-0.7).curveTo(-62.9,25.8,-54.2,54.4).curveTo(-45.5,83.1,-19.3,97.7).lineTo(-75.5,198.9).curveTo(-142.8,161.4,-165.1,87.8).closePath();
	var mask_graphics_220 = new cjs.Graphics().moveTo(-165.6,87.3).curveTo(-187.9,13.3,-151.8,-55).curveTo(-115.7,-123.3,-42.3,-147.3).curveTo(31.3,-171.1,100.6,-136.3).curveTo(169.9,-101.4,195.3,-28.6).lineTo(85.7,9.1).curveTo(75.9,-19.3,48.9,-32.8).curveTo(21.9,-46.4,-6.7,-37.2).curveTo(-35.2,-27.8,-49.3,-1.3).curveTo(-63.4,25.4,-54.7,54.2).curveTo(-46.1,83.1,-19.8,97.7).lineTo(-76,198.9).curveTo(-143.3,161.4,-165.6,87.3).closePath();
	var mask_graphics_221 = new cjs.Graphics().moveTo(-166.7,86.6).curveTo(-188.6,12.4,-151.9,-56.3).curveTo(-115.1,-124.8,-40.9,-148.1).curveTo(33.5,-171.2,102.6,-135.4).curveTo(171.6,-99.5,195.7,-25.5).lineTo(85.6,10.4).curveTo(76.3,-18.5,49.3,-32.4).curveTo(22.5,-46.4,-6.5,-37.4).curveTo(-35.3,-28.4,-49.7,-1.7).curveTo(-63.9,25.1,-55.3,54).curveTo(-46.8,82.8,-20.3,97.7).lineTo(-76.5,198.9).curveTo(-144.6,161,-166.7,86.6).closePath();
	var mask_graphics_222 = new cjs.Graphics().moveTo(-167.1,86.2).curveTo(-189,11.4,-151.6,-57.2).curveTo(-114.1,-125.9,-39.4,-148.5).curveTo(35.5,-171,104.3,-133.9).curveTo(173.3,-96.8,196.3,-22.3).lineTo(85.5,11.6).curveTo(76.6,-17.4,49.7,-31.9).curveTo(22.9,-46.4,-6.2,-37.6).curveTo(-35.3,-28.8,-49.9,-2.1).curveTo(-64.4,24.7,-55.9,53.7).curveTo(-47.4,82.8,-20.8,97.7).lineTo(-77,198.9).curveTo(-145.1,161,-167.1,86.2).closePath();
	var mask_graphics_223 = new cjs.Graphics().moveTo(-167.8,85.3).curveTo(-189.5,9.8,-151.4,-58.8).curveTo(-113.2,-127.5,-37.8,-149.2).curveTo(37.8,-170.8,106.4,-132.8).curveTo(175.1,-94.6,196.7,-19.2).lineTo(85.4,12.8).curveTo(77,-16.6,50.2,-31.5).curveTo(23.5,-46.2,-5.8,-37.8).curveTo(-35.2,-29.5,-50.1,-2.8).curveTo(-64.9,24,-56.5,53.4).curveTo(-48.1,82.8,-21.3,97.7).lineTo(-77.5,198.9).curveTo(-146.1,160.8,-167.8,85.3).closePath();
	var mask_graphics_224 = new cjs.Graphics().moveTo(-160.1,108).curveTo(-185.7,48.8,-169.7,-13.7).curveTo(-153.6,-76.2,-102.5,-116.3).curveTo(-51.4,-156.2,12.8,-156.6).curveTo(77,-157,128.6,-117.7).curveTo(180.4,-78.3,197.2,-15.9).lineTo(85.3,14).curveTo(78.8,-10.2,58.6,-25.5).curveTo(38.6,-40.8,13.5,-40.7).curveTo(-11.4,-40.6,-31.3,-25).curveTo(-51.2,-9.4,-57.4,14.8).curveTo(-63.7,39.2,-53.8,62.2).curveTo(-43.8,85.4,-21.7,97.7).lineTo(-77.9,198.9).curveTo(-134.4,167.3,-160.1,108).closePath();
	var mask_graphics_225 = new cjs.Graphics().moveTo(-160.6,107.6).curveTo(-186.2,48,-169.7,-14.8).curveTo(-153.2,-77.7,-101.6,-117.2).curveTo(-50,-156.8,14.6,-156.5).curveTo(79.2,-156.2,130.5,-115.9).curveTo(181.8,-75.5,197.6,-12.8).lineTo(85.3,15.2).curveTo(79.1,-9.1,59,-24.8).curveTo(39.1,-40.6,13.9,-40.7).curveTo(-11.1,-40.8,-31.2,-25.4).curveTo(-51.2,-10,-57.7,14.4).curveTo(-64.2,38.9,-54.2,62.1).curveTo(-44.2,85.4,-22.1,97.7).lineTo(-78.3,198.9).curveTo(-134.9,167.3,-160.6,107.6).closePath();
	var mask_graphics_226 = new cjs.Graphics().moveTo(-161.3,107.2).curveTo(-186.8,47.1,-170,-16).curveTo(-153.1,-79,-100.9,-118.3).curveTo(-48.8,-157.6,16.5,-156.5).curveTo(81.8,-155.3,132.6,-114.2).curveTo(183.3,-73.2,198,-9.5).lineTo(85.2,16.5).curveTo(79.5,-8.2,59.7,-24.3).curveTo(39.9,-40.3,14.5,-40.7).curveTo(-10.9,-41.1,-31.2,-25.8).curveTo(-51.5,-10.5,-58.1,14).curveTo(-64.6,38.5,-54.7,62).curveTo(-44.7,85.4,-22.5,97.7).lineTo(-78.7,198.9).curveTo(-135.7,167.3,-161.3,107.2).closePath();
	var mask_graphics_227 = new cjs.Graphics().moveTo(-161.9,106.5).curveTo(-187.4,46.2,-170,-16.9).curveTo(-152.6,-80,-99.8,-119.2).curveTo(-46.8,-158.4,18.3,-156.5).curveTo(83.5,-154.6,134,-112.5).curveTo(184.6,-70.4,198.3,-6.3).lineTo(85.1,17.8).curveTo(79.8,-7.1,60,-23.6).curveTo(40.4,-40,14.9,-40.7).curveTo(-10.4,-41.4,-31,-26.2).curveTo(-51.6,-10.9,-58.3,13.6).curveTo(-65.1,38.3,-55.2,61.7).curveTo(-45.2,85.3,-22.8,97.7).lineTo(-79,198.9).curveTo(-136.3,166.9,-161.9,106.5).closePath();
	var mask_graphics_228 = new cjs.Graphics().moveTo(-162.4,106.1).curveTo(-187.9,45.4,-170,-18.1).curveTo(-152,-81.5,-98.7,-120.2).curveTo(-45.4,-158.9,20,-156.3).curveTo(85.6,-153.8,135.8,-110.6).curveTo(186,-67.4,198.6,-3).lineTo(85,19).curveTo(80.1,-6,60.5,-22.8).curveTo(41,-39.6,15.4,-40.7).curveTo(-10,-41.6,-30.8,-26.6).curveTo(-51.5,-11.4,-58.5,13.2).curveTo(-65.4,37.9,-55.5,61.6).curveTo(-45.5,85.3,-23.2,97.7).lineTo(-79.4,198.9).curveTo(-136.8,166.9,-162.4,106.1).closePath();
	var mask_graphics_229 = new cjs.Graphics().moveTo(-163,105.6).curveTo(-188.3,44.5,-170.1,-19.2).curveTo(-151.8,-82.8,-97.9,-121.3).curveTo(-44,-159.7,22.1,-156.2).curveTo(88.3,-152.7,137.9,-108.9).curveTo(187.5,-65.1,199,0.2).lineTo(85,20.2).curveTo(80.5,-5.1,61.1,-22.1).curveTo(41.9,-39.2,16,-40.6).curveTo(-9.7,-41.9,-30.7,-27).curveTo(-51.6,-12,-58.7,12.8).curveTo(-65.8,37.6,-56,61.3).curveTo(-46.1,85.1,-23.5,97.7).lineTo(-79.7,198.9).curveTo(-137.5,166.8,-163,105.6).closePath();
	var mask_graphics_230 = new cjs.Graphics().moveTo(-163.5,105).curveTo(-188.9,43.7,-170.1,-20.1).curveTo(-151.2,-83.8,-96.7,-122.1).curveTo(-41.9,-160.4,24,-156.2).curveTo(90,-152,139.3,-107.1).curveTo(188.7,-62.1,199.3,3.5).lineTo(84.8,21.6).curveTo(80.8,-4,61.5,-21.5).curveTo(42.3,-38.9,16.6,-40.6).curveTo(-9,-42.2,-30.3,-27.3).curveTo(-51.6,-12.4,-58.9,12.4).curveTo(-66.2,37.2,-56.3,61).curveTo(-46.4,85,-23.8,97.7).lineTo(-80,198.9).curveTo(-138.1,166.5,-163.5,105).closePath();
	var mask_graphics_231 = new cjs.Graphics().moveTo(-163.8,104.5).curveTo(-189.1,42.7,-169.8,-21.2).curveTo(-150.5,-85.1,-95.5,-123).curveTo(-40.4,-160.9,25.8,-155.9).curveTo(92.2,-150.9,141.1,-105).curveTo(190.1,-59.1,199.5,6.7).lineTo(84.9,22.8).curveTo(81.2,-2.8,62.1,-20.6).curveTo(43,-38.5,17.2,-40.6).curveTo(-8.6,-42.5,-30,-27.7).curveTo(-51.4,-12.9,-58.9,12).curveTo(-66.4,36.9,-56.6,60.9).curveTo(-46.7,85,-24,97.7).lineTo(-80.2,198.9).curveTo(-138.4,166.4,-163.8,104.5).closePath();
	var mask_graphics_232 = new cjs.Graphics().moveTo(-164.3,104.1).curveTo(-189.5,41.9,-169.8,-22.3).curveTo(-150.2,-86.5,-94.6,-124.1).curveTo(-38.9,-161.6,28,-155.8).curveTo(94.9,-149.8,143.1,-103.3).curveTo(191.5,-56.7,199.7,10).lineTo(84.8,24.2).curveTo(81.5,-1.8,62.7,-20).curveTo(44,-38.1,17.9,-40.4).curveTo(-8.2,-42.7,-29.8,-28.1).curveTo(-51.5,-13.5,-59.2,11.4).curveTo(-66.8,36.5,-57,60.7).curveTo(-47.2,85,-24.3,97.7).lineTo(-80.5,198.9).curveTo(-139.1,166.4,-164.3,104.1).closePath();
	var mask_graphics_233 = new cjs.Graphics().moveTo(-164.7,103.4).curveTo(-189.9,41,-169.7,-23.4).curveTo(-149.5,-87.6,-93.2,-124.9).curveTo(-36.7,-162.3,29.9,-155.7).curveTo(96.7,-149,144.7,-101.4).curveTo(192.8,-53.7,199.9,13.3).lineTo(84.8,25.4).curveTo(82,-0.6,63.2,-19.3).curveTo(44.5,-37.8,18.5,-40.4).curveTo(-7.3,-43,-29.3,-28.5).curveTo(-51.2,-13.9,-59.1,11.2).curveTo(-66.9,36.2,-57.2,60.5).curveTo(-47.4,84.8,-24.4,97.7).lineTo(-80.6,198.9).curveTo(-139.5,166,-164.7,103.4).closePath();
	var mask_graphics_234 = new cjs.Graphics().moveTo(-165,103).curveTo(-190.1,40.2,-169.5,-24.4).curveTo(-148.8,-88.9,-91.9,-125.9).curveTo(-35,-162.7,31.9,-155.4).curveTo(98.9,-147.9,146.4,-99.3).curveTo(194.1,-50.6,200.1,16.6).lineTo(84.7,26.6).curveTo(82.4,0.6,63.8,-18.3).curveTo(45.3,-37.3,19.1,-40.3).curveTo(-6.9,-43.1,-29.1,-28.8).curveTo(-51.1,-14.4,-59.3,10.6).curveTo(-67.3,35.8,-57.5,60.3).curveTo(-47.6,84.8,-24.6,97.7).lineTo(-80.8,198.9).curveTo(-139.9,166,-165,103).closePath();
	var mask_graphics_235 = new cjs.Graphics().moveTo(-165.4,102.6).curveTo(-190.4,39.2,-169.4,-25.5).curveTo(-148.2,-90.3,-90.8,-126.8).curveTo(-33.4,-163.4,34.2,-155.1).curveTo(101.7,-146.7,148.6,-97.4).curveTo(195.5,-48,200.2,19.8).lineTo(84.7,28).curveTo(82.8,1.6,64.5,-17.7).curveTo(46.4,-36.9,19.9,-40.2).curveTo(-6.3,-43.4,-28.7,-29.2).curveTo(-51,-15,-59.3,10.2).curveTo(-67.4,35.6,-57.6,60.2).curveTo(-47.9,84.8,-24.7,97.7).lineTo(-80.9,198.9).curveTo(-140.4,166,-165.4,102.6).closePath();
	var mask_graphics_236 = new cjs.Graphics().moveTo(-165.8,101.9).curveTo(-190.7,38.4,-169.2,-26.5).curveTo(-147.5,-91.2,-89.4,-127.6).curveTo(-31.2,-164.1,36.1,-155).curveTo(103.4,-145.8,150,-95.4).curveTo(196.6,-44.9,200.3,23.2).lineTo(84.6,29.2).curveTo(83.3,2.8,65.1,-16.9).curveTo(47,-36.5,20.7,-40.2).curveTo(-5.4,-43.7,-28.1,-29.5).curveTo(-50.7,-15.2,-59.2,10).curveTo(-67.6,35.1,-57.9,59.9).curveTo(-48.1,84.7,-24.8,97.7).lineTo(-81,198.9).curveTo(-140.7,165.6,-165.8,101.9).closePath();
	var mask_graphics_237 = new cjs.Graphics().moveTo(-165.9,101.5).curveTo(-190.8,37.6,-168.7,-27.6).curveTo(-146.5,-92.6,-88,-128.6).curveTo(-29.5,-164.5,38.1,-154.6).curveTo(105.8,-144.6,151.9,-93.2).curveTo(197.9,-41.8,200.3,26.5).lineTo(84.7,30.5).curveTo(83.7,4,65.7,-16).curveTo(47.8,-36.1,21.4,-40).curveTo(-4.8,-43.8,-27.7,-29.9).curveTo(-50.5,-15.8,-59.1,9.5).curveTo(-67.7,34.9,-57.9,59.8).curveTo(-48.2,84.7,-24.9,97.7).lineTo(-81.1,198.9).curveTo(-140.9,165.6,-165.9,101.5).closePath();
	var mask_graphics_238 = new cjs.Graphics().moveTo(-166.2,101).curveTo(-191,36.6,-168.5,-28.6).curveTo(-146,-93.9,-86.8,-129.4).curveTo(-27.7,-164.9,40.5,-154.2).curveTo(108.6,-143.3,153.8,-91.3).curveTo(199.2,-39.2,200.4,29.9).lineTo(84.6,31.8).curveTo(84.1,4.9,66.5,-15.4).curveTo(48.9,-35.5,22.3,-39.7).curveTo(-4.2,-43.9,-27.2,-30.1).curveTo(-50.3,-16.3,-59.1,9).curveTo(-67.7,34.5,-58.1,59.5).curveTo(-48.5,84.6,-24.9,97.7).lineTo(-81.1,198.9).curveTo(-141.4,165.4,-166.2,101).closePath();
	var mask_graphics_239 = new cjs.Graphics().moveTo(-166.5,100.4).curveTo(-191.2,35.8,-168.2,-29.6).curveTo(-145.1,-94.9,-85.2,-130.2).curveTo(-25.3,-165.6,42.5,-153.9).curveTo(110.5,-142.3,155.4,-89.2).curveTo(200.4,-36.1,200.4,33.1).lineTo(84.6,33.1).curveTo(84.6,6.2,67.1,-14.6).curveTo(49.7,-35.1,23.1,-39.7).curveTo(-3.3,-44.2,-26.6,-30.5).curveTo(-49.9,-16.7,-58.9,8.7).curveTo(-67.9,34.2,-58.3,59.2).curveTo(-48.5,84.4,-24.9,97.7).lineTo(-81.1,198.9).curveTo(-141.7,165.2,-166.5,100.4).closePath();
	var mask_graphics_240 = new cjs.Graphics().moveTo(-166.2,100.1).curveTo(-190.9,35,-167.6,-30.6).curveTo(-144.4,-96.2,-84,-131).curveTo(-23.7,-165.7,44.5,-153.6).curveTo(112.8,-141.5,157.1,-87.3).curveTo(201.4,-33.2,200.3,36.3).lineTo(84.6,34.3).curveTo(85,7.2,67.8,-13.9).curveTo(50.5,-34.9,24,-39.7).curveTo(-2.6,-44.4,-26.1,-30.9).curveTo(-49.6,-17.3,-58.6,8.2).curveTo(-67.8,33.7,-58.1,59.1).curveTo(-48.5,84.5,-25,97.6).lineTo(-81.1,198.8).curveTo(-141.6,165.3,-166.2,100.1).closePath();
	var mask_graphics_241 = new cjs.Graphics().moveTo(-166.7,99.4).curveTo(-191.3,33.9,-167.4,-31.8).curveTo(-143.5,-97.5,-82.6,-132.1).curveTo(-21.7,-166.4,47,-153.1).curveTo(115.6,-139.7,159.2,-85.1).curveTo(202.7,-30.3,200.2,39.5).lineTo(84.6,35.7).curveTo(85.5,8.3,68.6,-13).curveTo(51.6,-34.2,24.9,-39.4).curveTo(-1.9,-44.6,-25.6,-31.3).curveTo(-49.3,-17.8,-58.5,7.8).curveTo(-67.9,33.4,-58.3,58.8).curveTo(-48.8,84.4,-25,97.6).lineTo(-81.1,198.8).curveTo(-142.2,165,-166.7,99.4).closePath();
	var mask_graphics_242 = new cjs.Graphics().moveTo(-166.8,99).curveTo(-191.3,33.1,-167.1,-32.7).curveTo(-142.9,-98.4,-81.2,-132.7).curveTo(-19.5,-166.8,48.9,-152.8).curveTo(117.4,-138.9,160.5,-83.1).curveTo(203.6,-27.4,200.1,42.9).lineTo(84.5,36.9).curveTo(85.9,9.6,69.1,-12.2).curveTo(52.4,-33.9,25.8,-39.3).curveTo(-0.9,-44.7,-25,-31.4).curveTo(-49.1,-18.1,-58.5,7.5).curveTo(-67.9,33.1,-58.4,58.6).curveTo(-48.9,84.3,-25,97.6).lineTo(-81.1,198.8).curveTo(-142.5,164.8,-166.8,99).closePath();
	var mask_graphics_243 = new cjs.Graphics().moveTo(-166.9,98.5).curveTo(-191.4,32.3,-166.6,-33.7).curveTo(-141.9,-99.7,-79.7,-133.4).curveTo(-17.7,-167,51,-152.3).curveTo(119.6,-137.5,162.2,-80.8).curveTo(204.6,-24,199.9,46.2).lineTo(84.4,38.2).curveTo(86.2,10.9,69.7,-11.3).curveTo(53.2,-33.4,26.4,-39.1).curveTo(-0.3,-44.8,-24.4,-31.7).curveTo(-48.7,-18.7,-58.3,7).curveTo(-67.9,32.8,-58.3,58.5).curveTo(-48.9,84.3,-25,97.6).lineTo(-81.1,198.8).curveTo(-142.5,164.8,-166.9,98.5).closePath();
	var mask_graphics_244 = new cjs.Graphics().moveTo(-167.3,97.8).curveTo(-191.7,31.2,-166.1,-35).curveTo(-140.8,-101.1,-78.2,-134.4).curveTo(-15.7,-167.7,53.4,-151.7).curveTo(122.4,-135.8,164.2,-78.4).curveTo(205.8,-21.2,199.6,49.6).lineTo(84.3,39.4).curveTo(86.7,11.9,70.5,-10.3).curveTo(54.3,-32.7,27.4,-38.9).curveTo(0.5,-45.1,-23.9,-32.2).curveTo(-48.3,-19.2,-58.1,6.6).curveTo(-68,32.4,-58.5,58.3).curveTo(-49.2,84.2,-25,97.6).lineTo(-81.1,198.8).curveTo(-143.1,164.5,-167.3,97.8).closePath();
	var mask_graphics_245 = new cjs.Graphics().moveTo(-167.4,97.5).curveTo(-191.6,30.5,-165.9,-35.7).curveTo(-140.2,-101.9,-76.8,-134.8).curveTo(-13.4,-167.9,55.5,-151.4).curveTo(124.2,-134.8,165.4,-76.5).curveTo(206.6,-18.1,199.3,52.8).lineTo(84.2,40.7).curveTo(86.9,13.2,70.9,-9.7).curveTo(54.8,-32.3,28.1,-38.8).curveTo(1.3,-45.2,-23.3,-32.3).curveTo(-48.1,-19.5,-58,6.2).curveTo(-68,32.1,-58.6,58.1).curveTo(-49.2,84.2,-25,97.6).lineTo(-81.1,198.8).curveTo(-143.3,164.4,-167.4,97.5).closePath();
	var mask_graphics_246 = new cjs.Graphics().moveTo(-167.4,97).curveTo(-191.6,29.7,-165.3,-36.8).curveTo(-139.1,-103.3,-75.3,-135.7).curveTo(-11.6,-168.1,57.5,-150.7).curveTo(126.5,-133.4,167,-74.1).curveTo(207.5,-14.7,198.9,56.1).lineTo(84,42).curveTo(87.3,14.4,71.6,-8.7).curveTo(55.8,-31.8,29.1,-38.6).curveTo(2.1,-45.3,-22.6,-32.7).curveTo(-47.6,-20.1,-57.8,5.9).curveTo(-68.1,31.7,-58.6,57.9).curveTo(-49.2,84.2,-25,97.6).lineTo(-81.1,198.8).curveTo(-143.3,164.4,-167.4,97).closePath();
	var mask_graphics_247 = new cjs.Graphics().moveTo(-167.9,96.3).curveTo(-192,28.6,-165,-38).curveTo(-138.2,-104.6,-73.8,-136.6).curveTo(-9.6,-168.7,59.8,-150.1).curveTo(129.1,-131.6,168.8,-71.6).curveTo(208.4,-11.8,198.4,59.3).lineTo(83.8,43.3).curveTo(87.7,15.6,72.3,-7.8).curveTo(56.7,-31.1,29.8,-38.3).curveTo(2.8,-45.5,-22.2,-33.1).curveTo(-47.2,-20.6,-57.7,5.3).curveTo(-68.2,31.2,-58.8,57.6).curveTo(-49.5,84,-25.1,97.6).lineTo(-81.2,198.8).curveTo(-144,164.1,-167.9,96.3).closePath();
	var mask_graphics_248 = new cjs.Graphics().moveTo(-168.1,95.9).curveTo(-192,27.8,-164.7,-38.8).curveTo(-137.5,-105.5,-72.4,-137.2).curveTo(-7.3,-168.9,61.8,-149.7).curveTo(130.9,-130.5,170,-69.6).curveTo(209,-8.6,198,62.7).lineTo(83.6,44.6).curveTo(87.9,16.8,72.8,-6.9).curveTo(57.5,-30.7,30.6,-38.1).curveTo(3.7,-45.7,-21.6,-33.3).curveTo(-47,-21,-57.5,5).curveTo(-68.2,30.9,-58.9,57.5).curveTo(-49.6,84,-25.1,97.6).lineTo(-81.2,198.8).curveTo(-144.2,163.9,-168.1,95.9).closePath();
	var mask_graphics_249 = new cjs.Graphics().moveTo(-168,95.5).curveTo(-192.1,27,-164.1,-39.9).curveTo(-136.4,-106.7,-70.9,-137.9).curveTo(-5.5,-169,63.7,-149.1).curveTo(133,-129,171.4,-67.1).curveTo(209.8,-5.3,197.4,65.9).lineTo(83.3,45.9).curveTo(88.2,18.1,73.3,-6).curveTo(58.3,-30.1,31.4,-37.9).curveTo(4.5,-45.7,-21,-33.5).curveTo(-46.6,-21.4,-57.3,4.6).curveTo(-68.2,30.6,-58.8,57.3).curveTo(-49.6,84,-25.1,97.6).lineTo(-81.2,198.8).curveTo(-144.2,163.9,-168,95.5).closePath();
	var mask_graphics_250 = new cjs.Graphics().moveTo(-168.5,94.8).curveTo(-192.3,26,-163.8,-41.1).curveTo(-135.5,-108,-69.5,-138.8).curveTo(-3.5,-169.6,66.1,-148.4).curveTo(135.7,-127,173.2,-64.7).curveTo(210.6,-2.3,196.7,69.1).lineTo(83,47.1).curveTo(88.5,19.3,74,-5).curveTo(59.3,-29.4,32.3,-37.7).curveTo(5.2,-45.9,-20.5,-33.9).curveTo(-46.2,-21.9,-57.3,4.2).curveTo(-68.3,30.3,-59,57.1).curveTo(-49.9,83.9,-25.1,97.6).lineTo(-81.2,198.8).curveTo(-144.8,163.6,-168.5,94.8).closePath();
	var mask_graphics_251 = new cjs.Graphics().moveTo(-168.7,94.3).curveTo(-192.3,25.2,-163.5,-41.8).curveTo(-134.7,-108.9,-67.9,-139.3).curveTo(-1.2,-169.7,68.2,-147.8).curveTo(137.3,-125.9,174.3,-62.5).curveTo(211.2,0.9,196.1,72.5).lineTo(82.8,48.4).curveTo(88.7,20.5,74.3,-4.2).curveTo(60,-28.8,33.1,-37.4).curveTo(6.1,-45.9,-19.8,-34).curveTo(-45.9,-22.2,-57.1,3.8).curveTo(-68.3,30,-59.1,56.9).curveTo(-50,83.8,-25.1,97.6).lineTo(-81.2,198.8).curveTo(-145.1,163.5,-168.7,94.3).closePath();
	var mask_graphics_252 = new cjs.Graphics().moveTo(-168.6,93.9).curveTo(-192.3,24.3,-162.9,-42.9).curveTo(-133.6,-110.1,-66.5,-139.9).curveTo(0.6,-169.8,70.1,-147.1).curveTo(139.5,-124.3,175.6,-59.9).curveTo(211.7,4.3,195.4,75.6).lineTo(82.6,49.6).curveTo(89,21.9,75,-3.2).curveTo(60.8,-28.2,33.9,-37.1).curveTo(6.7,-45.9,-19.3,-34.4).curveTo(-45.5,-22.7,-56.9,3.5).curveTo(-68.3,29.7,-59.1,56.7).curveTo(-50,83.8,-25.1,97.6).lineTo(-81.2,198.8).curveTo(-145.1,163.5,-168.6,93.9).closePath();
	var mask_graphics_253 = new cjs.Graphics().moveTo(-169,93.2).curveTo(-192.5,23.3,-162.5,-44).curveTo(-132.6,-111.4,-64.9,-140.8).curveTo(2.6,-170.2,72.4,-146.3).curveTo(142,-122.2,177.2,-57.5).curveTo(212.3,7.3,194.5,78.9).lineTo(82.2,50.8).curveTo(89.2,23,75.5,-2.2).curveTo(61.8,-27.5,34.7,-36.8).curveTo(7.6,-46.1,-18.8,-34.7).curveTo(-45.1,-23.2,-56.7,3).curveTo(-68.4,29.3,-59.3,56.5).curveTo(-50.3,83.7,-25.1,97.6).lineTo(-81.3,198.8).curveTo(-145.7,163.1,-169,93.2).closePath();
	var mask_graphics_254 = new cjs.Graphics().moveTo(-169.2,92.7).curveTo(-192.5,22.5,-162.1,-44.9).curveTo(-132,-112.3,-63.4,-141.3).curveTo(5,-170.3,74.3,-145.6).curveTo(143.7,-121,178.3,-55.2).curveTo(212.8,10.5,193.7,82).lineTo(82,52.1).curveTo(89.3,24.3,76,-1.4).curveTo(62.5,-27,35.4,-36.5).curveTo(8.4,-46.1,-18.2,-34.8).curveTo(-44.8,-23.6,-56.6,2.7).curveTo(-68.4,29,-59.3,56.4).curveTo(-50.2,83.7,-25.1,97.6).lineTo(-81.2,198.8).curveTo(-145.9,163,-169.2,92.7).closePath();
	var mask_graphics_255 = new cjs.Graphics().moveTo(-169.2,92.4).curveTo(-192.5,21.7,-161.6,-45.9).curveTo(-130.7,-113.4,-62,-141.9).curveTo(6.8,-170.4,76.2,-144.9).curveTo(145.7,-119.3,179.5,-52.7).curveTo(213.2,14,192.9,85.2).lineTo(81.6,53.3).curveTo(89.5,25.6,76.4,-0.4).curveTo(63.2,-26.3,36.2,-36.3).curveTo(9.1,-46.1,-17.6,-35.1).curveTo(-44.3,-23.9,-56.4,2.2).curveTo(-68.5,28.5,-59.4,56).curveTo(-50.4,83.6,-25.1,97.6).lineTo(-81.2,198.8).curveTo(-145.9,163,-169.2,92.4).closePath();
	var mask_graphics_256 = new cjs.Graphics().moveTo(-169.7,91.7).curveTo(-192.8,20.6,-161.2,-47).curveTo(-129.8,-114.7,-60.5,-142.7).curveTo(8.8,-170.8,78.5,-144.1).curveTo(148.1,-117.2,180.9,-50.1).curveTo(213.7,17,191.8,88.4).lineTo(81.1,54.6).curveTo(89.6,26.8,77,0.6).curveTo(64.2,-25.5,37.1,-35.9).curveTo(9.9,-46.3,-17,-35.4).curveTo(-44.1,-24.5,-56.4,1.8).curveTo(-68.6,28.1,-59.6,55.7).curveTo(-50.7,83.5,-25.2,97.6).lineTo(-81.3,198.8).curveTo(-146.6,162.7,-169.7,91.7).closePath();
	var mask_graphics_257 = new cjs.Graphics().moveTo(-169.8,91.1).curveTo(-192.8,19.9,-160.8,-47.9).curveTo(-128.9,-115.5,-58.9,-143.1).curveTo(11.1,-170.7,80.5,-143.4).curveTo(149.8,-116,181.9,-47.9).curveTo(214,20.3,190.9,91.6).lineTo(80.8,55.8).curveTo(89.8,28.1,77.4,1.6).curveTo(64.9,-25,37.8,-35.7).curveTo(10.8,-46.3,-16.4,-35.5).curveTo(-43.7,-24.8,-56.1,1.5).curveTo(-68.6,27.8,-59.5,55.6).curveTo(-50.6,83.5,-25.2,97.6).lineTo(-81.3,198.8).curveTo(-146.8,162.5,-169.8,91.1).closePath();
	var mask_graphics_258 = new cjs.Graphics().moveTo(-169.7,90.7).curveTo(-192.8,18.9,-160.2,-49).curveTo(-127.8,-116.7,-57.4,-143.8).curveTo(13,-170.8,82.5,-142.5).curveTo(151.8,-114.2,183,-45.1).curveTo(214.2,23.8,189.9,94.7).lineTo(80.5,57).curveTo(89.9,29.4,77.8,2.6).curveTo(65.6,-24.3,38.6,-35.4).curveTo(11.5,-46.3,-15.8,-35.8).curveTo(-43.3,-25.3,-55.9,1.1).curveTo(-68.6,27.5,-59.6,55.4).curveTo(-50.6,83.5,-25.2,97.6).lineTo(-81.3,198.8).curveTo(-146.8,162.5,-169.7,90.7).closePath();
	var mask_graphics_259 = new cjs.Graphics().moveTo(-170.3,90).curveTo(-193,17.9,-159.9,-50.1).curveTo(-126.7,-118.1,-55.8,-144.6).curveTo(14.9,-171.1,84.7,-141.6).curveTo(154.2,-112,184.4,-42.7).curveTo(214.5,26.7,188.7,97.8).lineTo(79.8,58.3).curveTo(89.9,30.6,78.3,3.5).curveTo(66.4,-23.5,39.4,-34.9).curveTo(12.3,-46.4,-15.3,-36.1).curveTo(-42.9,-25.9,-55.7,0.6).curveTo(-68.7,27.2,-59.8,55.2).curveTo(-50.9,83.3,-25.2,97.6).lineTo(-81.4,198.8).curveTo(-147.6,162.2,-170.3,90).closePath();
	var mask_graphics_260 = new cjs.Graphics().moveTo(-170.3,89.6).curveTo(-193,17.1,-159.4,-50.8).curveTo(-125.9,-118.8,-54.2,-144.8).curveTo(17.4,-171,86.6,-140.8).curveTo(155.8,-110.5,185.3,-40.3).curveTo(214.7,30,187.5,100.9).lineTo(79.5,59.5).curveTo(90,31.8,78.6,4.4).curveTo(67.1,-22.8,40.2,-34.6).curveTo(13.3,-46.5,-14.6,-36.4).curveTo(-42.6,-26.2,-55.6,0.4).curveTo(-68.7,26.9,-59.8,55).curveTo(-51,83.3,-25.2,97.6).lineTo(-81.3,198.8).curveTo(-147.7,162.1,-170.3,89.6).closePath();
	var mask_graphics_261 = new cjs.Graphics().moveTo(-170.2,89.2).curveTo(-192.9,16.3,-158.7,-51.8).curveTo(-124.7,-119.9,-52.6,-145.5).curveTo(19.3,-171,88.6,-139.9).curveTo(157.7,-108.7,186.3,-37.6).curveTo(214.8,33.6,186.4,104).lineTo(79,60.7).curveTo(90,33.2,78.9,5.5).curveTo(67.8,-22.1,40.9,-34.3).curveTo(14,-46.4,-13.9,-36.5).curveTo(-42.1,-26.5,-55.3,-0.1).curveTo(-68.6,26.4,-59.7,54.9).curveTo(-51,83.3,-25.2,97.6).lineTo(-81.3,198.8).curveTo(-147.6,162.1,-170.2,89.2).closePath();
	var mask_graphics_262 = new cjs.Graphics().moveTo(-170.7,88.5).curveTo(-193.1,15.2,-158.4,-53).curveTo(-123.7,-121.2,-51.2,-146.2).curveTo(21.2,-171.2,90.7,-138.8).curveTo(160.1,-106.4,187.5,-35).curveTo(214.9,36.5,185,107.1).lineTo(78.5,61.8).curveTo(90.1,34.4,79.5,6.5).curveTo(68.8,-21.3,41.7,-33.9).curveTo(14.6,-46.5,-13.5,-36.8).curveTo(-41.7,-27,-55.2,-0.4).curveTo(-68.8,26.1,-60,54.6).curveTo(-51.3,83.1,-25.3,97.6).lineTo(-81.4,198.8).curveTo(-148.4,161.7,-170.7,88.5).closePath();
	var mask_graphics_263 = new cjs.Graphics().moveTo(-170.7,87.8).curveTo(-193.1,14.2,-157.9,-54.1).curveTo(-122.9,-122.3,-49.5,-146.8).curveTo(23.6,-171.3,92.7,-138.3).curveTo(161.5,-105.3,188.3,-32.9).curveTo(215,39.7,183.8,109.9).lineTo(78,62.7).curveTo(90.2,35.5,79.8,7.3).curveTo(69.3,-20.9,42.5,-33.8).curveTo(15.6,-46.7,-12.8,-37.1).curveTo(-41.3,-27.6,-54.9,-1).curveTo(-68.7,25.5,-60,54.2).curveTo(-51.4,82.8,-25.2,97.3).lineTo(-81.3,198.6).curveTo(-148.5,161.4,-170.7,87.8).closePath();
	var mask_graphics_264 = new cjs.Graphics().moveTo(-170.7,87.3).curveTo(-193,13.3,-157.2,-55.1).curveTo(-121.5,-123.4,-48,-147.4).curveTo(25.5,-171.2,94.5,-137.3).curveTo(163.4,-103.3,189.2,-30).curveTo(214.9,43.2,182.5,112.8).lineTo(77.5,64).curveTo(90.1,36.9,80.2,8.3).curveTo(70.1,-20.2,43.3,-33.5).curveTo(16.4,-46.7,-12.1,-37.4).curveTo(-40.8,-28.1,-54.7,-1.4).curveTo(-68.7,25.2,-60,54).curveTo(-51.4,82.8,-25.2,97.3).lineTo(-81.3,198.6).curveTo(-148.6,161.3,-170.7,87.3).closePath();
	var mask_graphics_265 = new cjs.Graphics().moveTo(-171.2,86.6).curveTo(-193.3,12.2,-156.9,-56.3).curveTo(-120.5,-124.7,-46.4,-148).curveTo(27.5,-171.3,96.7,-136.1).curveTo(165.7,-100.9,190.3,-27.4).curveTo(214.9,46.2,180.9,115.9).lineTo(76.9,65).curveTo(90.1,38,80.6,9.4).curveTo(70.9,-19.2,44,-33).curveTo(17.1,-46.7,-11.7,-37.6).curveTo(-40.5,-28.5,-54.6,-1.9).curveTo(-68.8,24.7,-60.2,53.7).curveTo(-51.7,82.7,-25.2,97.3).lineTo(-81.4,198.6).curveTo(-149.2,161,-171.2,86.6).closePath();
	var mask_graphics_266 = new cjs.Graphics().moveTo(-171.2,86.2).curveTo(-193.3,11.5,-156.4,-57).curveTo(-119.6,-125.4,-44.8,-148.4).curveTo(29.9,-171.2,98.6,-135.3).curveTo(167.1,-99.4,191,-25).curveTo(214.8,49.5,179.5,118.8).lineTo(76.4,66.3).curveTo(90,39.3,80.9,10.3).curveTo(71.6,-18.7,44.9,-32.8).curveTo(18,-46.7,-11,-37.9).curveTo(-40.1,-28.9,-54.4,-2.3).curveTo(-68.7,24.4,-60.1,53.5).curveTo(-51.6,82.7,-25.2,97.3).lineTo(-81.3,198.6).curveTo(-149.3,160.9,-171.2,86.2).closePath();
	var mask_graphics_267 = new cjs.Graphics().moveTo(-171.2,85.7).curveTo(-193.1,10.6,-155.7,-58).curveTo(-118.3,-126.6,-43.2,-148.9).curveTo(31.8,-171.2,100.4,-134.2).curveTo(168.9,-97.3,191.8,-22.1).curveTo(214.6,53,178,121.8).lineTo(75.8,67.4).curveTo(90,40.7,81.2,11.3).curveTo(72.3,-17.9,45.5,-32.2).curveTo(18.8,-46.6,-10.3,-37.9).curveTo(-39.6,-29.2,-54,-2.6).curveTo(-68.7,24.1,-60.1,53.3).curveTo(-51.7,82.6,-25.2,97.3).lineTo(-81.3,198.6).curveTo(-149.4,160.9,-171.2,85.7).closePath();
	var mask_graphics_268 = new cjs.Graphics().moveTo(-171.7,85.1).curveTo(-193.4,9.6,-155.3,-59.1).curveTo(-117.2,-127.8,-41.8,-149.5).curveTo(33.7,-171.1,102.4,-133.1).curveTo(171.1,-95,192.8,-19.5).curveTo(214.4,56,176.4,124.7).lineTo(75.1,68.5).curveTo(89.9,41.8,81.6,12.4).curveTo(73.1,-17,46.3,-31.8).curveTo(19.5,-46.6,-9.7,-38.2).curveTo(-39.2,-29.8,-54,-3).curveTo(-68.9,23.8,-60.3,53.1).curveTo(-52,82.5,-25.2,97.3).lineTo(-81.4,198.6).curveTo(-150.1,160.5,-171.7,85.1).closePath();
	var mask_graphics_269 = new cjs.Graphics().moveTo(-164.3,104.6).curveTo(-189.7,42.8,-170.9,-21).curveTo(-152.3,-84.7,-97.7,-122.7).curveTo(-43.2,-160.6,23.5,-156.4).curveTo(90.1,-152,139,-107).curveTo(187.8,-62,198,4).curveTo(208.1,70,175,127.5).lineTo(74.7,69.7).curveTo(87.6,47.3,83.7,21.6).curveTo(79.7,-4.1,60.7,-21.6).curveTo(41.7,-39.1,15.8,-40.8).curveTo(-10.2,-42.5,-31.4,-27.8).curveTo(-52.7,-13,-60,11.8).curveTo(-67.3,36.7,-57.4,60.8).curveTo(-47.6,84.8,-25,97.3).lineTo(-81.1,198.6).curveTo(-139.1,166.4,-164.3,104.6).closePath();
	var mask_graphics_270 = new cjs.Graphics().moveTo(-164.4,104.1).curveTo(-189.8,41.9,-170.7,-21.8).curveTo(-151.7,-85.5,-96.5,-123.3).curveTo(-41.5,-161.1,25.5,-156).curveTo(92.5,-150.9,140.9,-105.3).curveTo(189.1,-59.6,198.4,7).curveTo(207.5,73.5,173.3,130.4).lineTo(74.1,70.8).curveTo(87.4,48.6,83.9,22.7).curveTo(80.3,-3.2,61.6,-21).curveTo(42.6,-38.8,16.6,-40.8).curveTo(-9.5,-42.8,-31,-28).curveTo(-52.5,-13.3,-59.9,11.5).curveTo(-67.3,36.4,-57.4,60.6).curveTo(-47.6,84.8,-24.9,97.3).lineTo(-81,198.6).curveTo(-139.2,166.4,-164.4,104.1).closePath();
	var mask_graphics_271 = new cjs.Graphics().moveTo(-165.1,103.8).curveTo(-190.2,41.6,-170.5,-22.6).curveTo(-150.9,-86.8,-95.3,-124.4).curveTo(-39.7,-161.9,27.3,-156.1).curveTo(94.1,-150.3,142.4,-103.7).curveTo(190.7,-57,198.9,9.6).curveTo(207.1,76.2,171.5,133.2).lineTo(73.2,71.9).curveTo(87.1,49.7,83.9,23.8).curveTo(80.7,-2.2,61.9,-20.3).curveTo(43.2,-38.4,17.2,-40.7).curveTo(-8.9,-43,-30.5,-28.3).curveTo(-52.2,-13.7,-59.8,11.2).curveTo(-67.5,36.2,-57.7,60.4).curveTo(-47.9,84.7,-25.1,97.3).lineTo(-81.2,198.6).curveTo(-139.9,166,-165.1,103.8).closePath();
	var mask_graphics_272 = new cjs.Graphics().moveTo(-165.1,103.4).curveTo(-190.2,40.9,-170.4,-23.5).curveTo(-150.5,-87.7,-94.3,-125).curveTo(-38.2,-162.2,29.2,-155.8).curveTo(96.5,-149.4,144.2,-102).curveTo(191.9,-54.4,199.2,12.6).curveTo(206.5,79.6,169.8,136.1).lineTo(72.7,73).curveTo(86.9,51,84.2,24.8).curveTo(81.3,-1.2,62.7,-19.7).curveTo(44.2,-38.2,18,-40.6).curveTo(-8.3,-43.1,-30.2,-28.6).curveTo(-52,-14.1,-59.7,10.9).curveTo(-67.5,35.9,-57.6,60.2).curveTo(-47.9,84.7,-25,97.3).lineTo(-81.1,198.6).curveTo(-140,166,-165.1,103.4).closePath();
	var mask_graphics_273 = new cjs.Graphics().moveTo(-165,103).curveTo(-190.1,40,-170,-24.2).curveTo(-149.9,-88.5,-93.1,-125.6).curveTo(-36.5,-162.7,31.2,-155.5).curveTo(98.9,-148.3,146.1,-100.2).curveTo(193.2,-51.9,199.5,15.5).curveTo(205.6,83,168,138.8).lineTo(72,74.1).curveTo(86.6,52.3,84.2,26.1).curveTo(81.7,-0.2,63.5,-19).curveTo(45.1,-37.7,18.8,-40.5).curveTo(-7.6,-43.4,-29.6,-29).curveTo(-51.8,-14.4,-59.5,10.5).curveTo(-67.4,35.6,-57.6,60.1).curveTo(-48,84.6,-25,97.3).lineTo(-81.1,198.6).curveTo(-140,166,-165,103).closePath();
	var mask_graphics_274 = new cjs.Graphics().moveTo(-165,102.5).curveTo(-190.1,39.2,-169.4,-25.3).curveTo(-148.8,-89.8,-91.7,-126.4).curveTo(-34.7,-163.1,33,-155.3).curveTo(100.7,-147.5,147.5,-98.5).curveTo(194.4,-49.5,199.7,18.3).curveTo(204.9,86.3,166.1,141.4).lineTo(71.3,75.1).curveTo(86.3,53.5,84.4,27.1).curveTo(82.3,0.7,64.1,-18.4).curveTo(45.8,-37.3,19.5,-40.5).curveTo(-6.8,-43.4,-29,-29.2).curveTo(-51.3,-15,-59.3,10.2).curveTo(-67.4,35.2,-57.6,59.9).curveTo(-47.9,84.6,-24.9,97.3).lineTo(-81,198.6).curveTo(-140,165.9,-165,102.5).closePath();
	var mask_graphics_275 = new cjs.Graphics().moveTo(-165.5,102).curveTo(-190.5,38.4,-169.4,-26.2).curveTo(-148.5,-90.8,-90.8,-127.3).curveTo(-33.1,-163.7,34.9,-155.1).curveTo(102.9,-146.6,149.2,-96.9).curveTo(195.5,-47.2,199.9,20.8).curveTo(204.3,89,164.2,144.3).lineTo(70.5,76.1).curveTo(86.1,54.7,84.5,28.1).curveTo(82.7,1.6,64.8,-17.7).curveTo(46.7,-37,20.2,-40.4).curveTo(-6.3,-43.7,-28.7,-29.5).curveTo(-51.2,-15.4,-59.3,9.9).curveTo(-67.6,35,-57.9,59.7).curveTo(-48.2,84.5,-25,97.3).lineTo(-81.1,198.6).curveTo(-140.7,165.6,-165.5,102).closePath();
	var mask_graphics_276 = new cjs.Graphics().moveTo(-165.6,101.4).curveTo(-190.5,37.4,-168.9,-27.3).curveTo(-147.4,-92.1,-89.3,-128.2).curveTo(-31.3,-164.1,36.7,-155).curveTo(104.7,-145.8,150.7,-95.3).curveTo(196.7,-44.8,200,23.8).curveTo(203.4,92.4,162.2,146.8).lineTo(69.8,77.2).curveTo(85.8,56,84.5,29.3).curveTo(83.2,2.6,65.3,-17.1).curveTo(47.4,-36.8,21,-40.3).curveTo(-5.5,-43.8,-28.1,-29.9).curveTo(-50.8,-15.8,-59.2,9.4).curveTo(-67.6,34.7,-57.9,59.6).curveTo(-48.1,84.5,-24.9,97.3).lineTo(-81,198.6).curveTo(-140.7,165.5,-165.6,101.4).closePath();
	var mask_graphics_277 = new cjs.Graphics().moveTo(-165.6,101.1).curveTo(-190.5,36.7,-168.6,-28.1).curveTo(-146.8,-92.8,-88.1,-128.7).curveTo(-29.5,-164.5,38.8,-154.6).curveTo(107,-144.6,152.5,-93.4).curveTo(197.8,-42.3,200.2,26.7).curveTo(202.3,95.7,160.3,149.5).lineTo(69,78.1).curveTo(85.4,57.2,84.5,30.4).curveTo(83.6,3.5,66,-16.4).curveTo(48.3,-36.3,21.7,-40.2).curveTo(-4.9,-44.1,-27.7,-30.1).curveTo(-50.6,-16.1,-59,9.1).curveTo(-67.5,34.2,-57.7,59.3).curveTo(-48.1,84.5,-24.9,97.3).lineTo(-81,198.6).curveTo(-140.7,165.5,-165.6,101.1).closePath();
	var mask_graphics_278 = new cjs.Graphics().moveTo(-166.2,100.7).curveTo(-190.9,36.3,-168.4,-28.9).curveTo(-146,-94.2,-86.9,-129.7).curveTo(-27.8,-165.3,40.4,-154.5).curveTo(108.5,-143.7,153.9,-91.7).curveTo(199.1,-39.5,200.4,29.5).curveTo(201.5,98.4,158.1,152.1).lineTo(68.1,79.3).curveTo(85,58.3,84.6,31.4).curveTo(84.1,4.7,66.5,-15.6).curveTo(48.8,-35.9,22.3,-40.1).curveTo(-4.3,-44.3,-27.2,-30.5).curveTo(-50.3,-16.7,-58.9,8.7).curveTo(-67.7,34.2,-58.1,59.2).curveTo(-48.5,84.3,-25,97.3).lineTo(-81.1,198.6).curveTo(-141.5,165.1,-166.2,100.7).closePath();
	var mask_graphics_279 = new cjs.Graphics().moveTo(-166.3,100.3).curveTo(-191,35.6,-168.3,-29.7).curveTo(-145.6,-95,-85.9,-130.3).curveTo(-26.3,-165.5,42.1,-154.2).curveTo(110.3,-143,155.2,-90).curveTo(200.1,-36.8,200.4,32.4).curveTo(200.6,101.7,156,154.6).lineTo(67.3,80.2).curveTo(84.6,59.6,84.5,32.6).curveTo(84.4,5.6,67,-15).curveTo(49.5,-35.7,22.9,-40.1).curveTo(-3.6,-44.4,-26.8,-30.8).curveTo(-50.1,-17,-58.9,8.4).curveTo(-67.7,33.9,-58.1,59).curveTo(-48.6,84.3,-25,97.3).lineTo(-81.2,198.6).curveTo(-141.7,165.1,-166.3,100.3).closePath();
	var mask_graphics_280 = new cjs.Graphics().moveTo(-166.2,99.9).curveTo(-190.9,34.8,-167.9,-30.5).curveTo(-144.9,-95.8,-84.6,-130.8).curveTo(-24.5,-165.8,44.1,-153.8).curveTo(112.6,-141.7,156.9,-88).curveTo(201.1,-34.3,200.3,35.3).curveTo(199.5,105,153.9,157.1).lineTo(66.5,81.1).curveTo(84.2,60.9,84.6,33.7).curveTo(84.9,6.6,67.7,-14.3).curveTo(50.5,-35.2,23.8,-39.9).curveTo(-3,-44.6,-26.4,-30.9).curveTo(-49.9,-17.3,-58.7,8.2).curveTo(-67.6,33.6,-58,58.9).curveTo(-48.5,84.3,-25,97.3).lineTo(-81.1,198.6).curveTo(-141.6,165.1,-166.2,99.9).closePath();
	var mask_graphics_281 = new cjs.Graphics().moveTo(-166.2,99.5).curveTo(-190.9,34,-167.2,-31.6).curveTo(-143.8,-97.1,-83.2,-131.6).curveTo(-22.7,-166.2,45.8,-153.5).curveTo(114.3,-140.8,158.3,-86.2).curveTo(202.1,-31.7,200.2,38.2).curveTo(198.2,108.2,151.8,159.6).lineTo(65.7,82.2).curveTo(83.8,62.2,84.6,34.9).curveTo(85.4,7.6,68.3,-13.6).curveTo(51.2,-34.9,24.5,-39.8).curveTo(-2.2,-44.6,-25.8,-31.2).curveTo(-49.3,-17.8,-58.5,7.6).curveTo(-67.7,33.2,-58,58.7).curveTo(-48.4,84.3,-24.9,97.3).lineTo(-81,198.6).curveTo(-141.6,165,-166.2,99.5).closePath();
	var mask_graphics_282 = new cjs.Graphics().moveTo(-166.8,98.8).curveTo(-191.3,33.1,-167.4,-32.5).curveTo(-143.4,-98,-82.3,-132.4).curveTo(-21.2,-166.7,47.7,-153.1).curveTo(116.5,-139.6,159.8,-84.5).curveTo(203,-29.3,200.2,40.9).curveTo(197.3,110.9,149.3,162.1).lineTo(64.7,83.1).curveTo(83.3,63.2,84.6,35.9).curveTo(85.6,8.6,68.9,-12.9).curveTo(52,-34.3,25.1,-39.6).curveTo(-1.7,-44.9,-25.5,-31.6).curveTo(-49.3,-18.1,-58.6,7.3).curveTo(-67.9,32.8,-58.3,58.5).curveTo(-48.8,84.1,-25,97.3).lineTo(-81.1,198.6).curveTo(-142.4,164.7,-166.8,98.8).closePath();
	var mask_graphics_283 = new cjs.Graphics().moveTo(-166.8,98.5).curveTo(-191.2,32.3,-166.7,-33.6).curveTo(-142.3,-99.3,-80.8,-133.2).curveTo(-19.3,-167.1,49.5,-153).curveTo(118.2,-138.7,161.1,-82.8).curveTo(203.9,-26.7,200,43.8).curveTo(196.1,114.2,147.2,164.4).lineTo(63.9,84).curveTo(82.9,64.5,84.5,37).curveTo(86.1,9.6,69.4,-12.2).curveTo(52.6,-34,26,-39.5).curveTo(-0.9,-45,-24.7,-31.8).curveTo(-48.7,-18.7,-58.3,6.9).curveTo(-67.8,32.5,-58.2,58.3).curveTo(-48.9,84.1,-24.9,97.3).lineTo(-81,198.6).curveTo(-142.4,164.6,-166.8,98.5).closePath();
	var mask_graphics_284 = new cjs.Graphics().moveTo(-166.7,98).curveTo(-191.1,31.4,-166.4,-34.3).curveTo(-141.7,-100.1,-79.5,-133.8).curveTo(-17.4,-167.3,51.5,-152.3).curveTo(120.4,-137.4,162.6,-80.9).curveTo(204.7,-24.2,199.7,46.5).curveTo(194.7,117.3,145,166.8).lineTo(63.1,84.9).curveTo(82.5,65.6,84.4,38.1).curveTo(86.3,10.6,69.9,-11.5).curveTo(53.5,-33.4,26.7,-39.3).curveTo(-0.2,-45,-24.3,-31.9).curveTo(-48.6,-18.9,-58.1,6.6).curveTo(-67.8,32.2,-58.3,58.1).curveTo(-48.8,84.1,-24.9,97.3).lineTo(-81,198.6).curveTo(-142.4,164.6,-166.7,98).closePath();
	var mask_graphics_285 = new cjs.Graphics().moveTo(-167.3,97.6).curveTo(-191.7,31,-166.2,-35.2).curveTo(-140.8,-101.4,-78.3,-134.7).curveTo(-15.7,-168,53.3,-152).curveTo(122.4,-136,164.1,-78.7).curveTo(205.8,-21.4,199.6,49.2).curveTo(193.4,119.9,142.5,169.1).lineTo(62,85.8).curveTo(81.9,66.6,84.3,39.2).curveTo(86.7,11.7,70.5,-10.6).curveTo(54.2,-32.9,27.4,-39.1).curveTo(0.5,-45.4,-23.9,-32.4).curveTo(-48.3,-19.4,-58.1,6.4).curveTo(-68,32.2,-58.5,58).curveTo(-49.2,83.9,-25,97.3).lineTo(-81.2,198.6).curveTo(-143.1,164.3,-167.3,97.6).closePath();
	var mask_graphics_286 = new cjs.Graphics().moveTo(-167.4,97.2).curveTo(-191.7,30.4,-165.9,-35.9).curveTo(-140.3,-102.2,-77.2,-135.2).curveTo(-14.2,-168.1,55,-151.6).curveTo(124.1,-135.1,165.3,-76.8).curveTo(206.5,-18.4,199.3,52.3).curveTo(192.2,123.1,140,171.4).lineTo(61.1,86.7).curveTo(81.4,67.9,84.3,40.4).curveTo(86.9,12.9,70.9,-9.9).curveTo(54.9,-32.6,28,-38.9).curveTo(1.1,-45.4,-23.5,-32.6).curveTo(-48.1,-19.8,-58,6).curveTo(-68,31.9,-58.6,57.8).curveTo(-49.2,83.9,-25,97.3).lineTo(-81.1,198.6).curveTo(-143.3,164.1,-167.4,97.2).closePath();
	var mask_graphics_287 = new cjs.Graphics().moveTo(-167.4,96.7).curveTo(-191.6,29.5,-165.6,-36.8).curveTo(-139.6,-102.9,-76,-135.6).curveTo(-12.4,-168.3,57,-151.1).curveTo(126.3,-133.8,166.8,-74.8).curveTo(207.2,-15.8,199,55.2).curveTo(190.6,126.2,137.7,173.7).lineTo(60.1,87.5).curveTo(80.8,69.1,84,41.4).curveTo(87.2,13.8,71.5,-9.2).curveTo(55.7,-32.1,28.8,-38.8).curveTo(1.8,-45.4,-22.9,-32.8).curveTo(-47.7,-20.1,-57.9,5.7).curveTo(-68.1,31.5,-58.6,57.6).curveTo(-49.2,83.9,-25,97.3).lineTo(-81.1,198.6).curveTo(-143.3,164.1,-167.4,96.7).closePath();
	var mask_graphics_288 = new cjs.Graphics().moveTo(-167.5,96.4).curveTo(-191.6,28.7,-165,-37.7).curveTo(-138.4,-104.1,-74.4,-136.3).curveTo(-10.6,-168.5,58.7,-150.5).curveTo(127.9,-132.6,168,-72.9).curveTo(208,-13.1,198.5,58.1).curveTo(189.1,129.4,135.3,175.9).lineTo(59.3,88.4).curveTo(80.3,70.3,84,42.7).curveTo(87.6,14.9,72,-8.4).curveTo(56.4,-31.6,29.5,-38.6).curveTo(2.5,-45.5,-22.4,-33).curveTo(-47.3,-20.5,-57.6,5.4).curveTo(-68,31.2,-58.6,57.5).curveTo(-49.2,83.9,-24.9,97.3).lineTo(-81.1,198.6).curveTo(-143.4,164.1,-167.5,96.4).closePath();
	var mask_graphics_289 = new cjs.Graphics().moveTo(-168,95.7).curveTo(-192,27.8,-165.1,-38.6).curveTo(-138.2,-105,-73.6,-137).curveTo(-9,-169,60.5,-150.2).curveTo(129.9,-131.4,169.3,-71).curveTo(208.6,-10.7,198.4,60.3).curveTo(188.2,131.3,132.5,178).lineTo(58.2,89.3).curveTo(79.8,71.2,83.8,43.6).curveTo(87.7,15.8,72.4,-7.7).curveTo(57.1,-31.2,30.1,-38.5).curveTo(3,-45.8,-22.1,-33.4).curveTo(-47.3,-20.8,-57.7,5).curveTo(-68.3,30.8,-58.9,57.2).curveTo(-49.6,83.7,-25.1,97.3).lineTo(-81.2,198.6).curveTo(-144.1,163.7,-168,95.7).closePath();
	var mask_graphics_290 = new cjs.Graphics().moveTo(-168.1,95.3).curveTo(-192,27,-164.5,-39.7).curveTo(-137.1,-106.2,-72.1,-137.7).curveTo(-7.2,-169.1,62.2,-149.7).curveTo(131.5,-130.3,170.4,-69.1).curveTo(209.3,-8,197.9,63.1).curveTo(186.5,134.3,130,180).lineTo(57.1,90.1).curveTo(79,72.3,83.5,44.6).curveTo(87.9,16.9,72.9,-6.9).curveTo(57.7,-30.6,30.8,-38.3).curveTo(3.8,-45.8,-21.4,-33.6).curveTo(-46.8,-21.4,-57.5,4.6).curveTo(-68.3,30.5,-58.9,57.1).curveTo(-49.6,83.7,-25.1,97.3).lineTo(-81.2,198.6).curveTo(-144.2,163.7,-168.1,95.3).closePath();
	var mask_graphics_291 = new cjs.Graphics().moveTo(-168,94.9).curveTo(-191.9,26.1,-164.1,-40.4).curveTo(-136.3,-106.9,-70.9,-138.2).curveTo(-5.4,-169.4,64.1,-149.1).curveTo(133.6,-128.7,171.8,-67).curveTo(209.9,-5.3,197.3,66).curveTo(184.7,137.4,127.5,182.2).lineTo(56.2,90.9).curveTo(78.4,73.5,83.4,45.8).curveTo(88.3,18,73.4,-6).curveTo(58.6,-30.1,31.6,-38.1).curveTo(4.5,-45.9,-21,-33.7).curveTo(-46.6,-21.6,-57.3,4.2).curveTo(-68.1,30.1,-58.8,56.9).curveTo(-49.6,83.7,-25,97.3).lineTo(-81.2,198.6).curveTo(-144.2,163.7,-168,94.9).closePath();
	var mask_graphics_292 = new cjs.Graphics().moveTo(-168.5,94.5).curveTo(-192.3,25.7,-163.8,-41.3).curveTo(-135.5,-108.3,-69.6,-139.1).curveTo(-3.6,-169.9,66.1,-148.6).curveTo(135.7,-127.2,173.2,-65).curveTo(210.6,-2.6,196.8,68.9).curveTo(182.9,140.3,124.8,184.1).lineTo(55,91.7).curveTo(77.6,74.7,83.1,46.9).curveTo(88.5,19.1,73.9,-5.2).curveTo(59.3,-29.6,32.3,-37.9).curveTo(5.1,-46.1,-20.5,-34.2).curveTo(-46.3,-22.1,-57.3,3.9).curveTo(-68.4,30,-59.1,56.8).curveTo(-49.9,83.6,-25.1,97.3).lineTo(-81.3,198.6).curveTo(-144.9,163.3,-168.5,94.5).closePath();
	var mask_graphics_293 = new cjs.Graphics().moveTo(-168.7,94.1).curveTo(-192.4,25,-163.6,-42.1).curveTo(-134.8,-109.1,-68.4,-139.5).curveTo(-2,-169.9,67.6,-148.1).curveTo(137.2,-126.2,174.1,-62.9).curveTo(211,0.5,196.4,71.6).curveTo(181.5,142.8,122.1,186.2).lineTo(54,92.4).curveTo(77.1,75.6,82.9,47.9).curveTo(88.6,20.3,74.2,-4.4).curveTo(59.9,-29.1,32.9,-37.6).curveTo(5.7,-46.1,-20.1,-34.2).curveTo(-46,-22.4,-57.1,3.6).curveTo(-68.4,29.7,-59.1,56.7).curveTo(-50,83.5,-25.2,97.3).lineTo(-81.3,198.6).curveTo(-145.1,163.2,-168.7,94.1).closePath();
	var mask_graphics_294 = new cjs.Graphics().moveTo(-168.6,93.7).curveTo(-192.4,24.2,-163.3,-42.9).curveTo(-134.2,-109.9,-67.2,-139.9).curveTo(-0.2,-170,69.6,-147.3).curveTo(139.2,-124.7,175.5,-60.7).curveTo(211.6,3.2,195.6,74.4).curveTo(179.6,145.8,119.3,188.1).lineTo(53,93.2).curveTo(76.4,76.8,82.6,49).curveTo(88.8,21.3,74.8,-3.7).curveTo(60.7,-28.5,33.6,-37.4).curveTo(6.4,-46.2,-19.6,-34.5).curveTo(-45.7,-22.8,-57,3.4).curveTo(-68.3,29.4,-59.2,56.5).curveTo(-50,83.5,-25.1,97.3).lineTo(-81.2,198.6).curveTo(-145.1,163.2,-168.6,93.7).closePath();
	var mask_graphics_295 = new cjs.Graphics().moveTo(-168.6,93.2).curveTo(-192.3,23.3,-162.5,-43.9).curveTo(-132.9,-110.9,-65.6,-140.5).curveTo(1.6,-170.1,71.2,-146.9).curveTo(140.8,-123.5,176.5,-58.8).curveTo(212,5.9,194.9,77.2).curveTo(177.7,148.7,116.7,189.9).lineTo(52.1,93.9).curveTo(75.8,78,82.5,50.1).curveTo(89.1,22.3,75.3,-2.8).curveTo(61.4,-28,34.3,-37.2).curveTo(7.1,-46.3,-18.9,-34.7).curveTo(-45.2,-23.1,-56.7,3).curveTo(-68.2,29.1,-59,56.4).curveTo(-49.9,83.5,-25.1,97.3).lineTo(-81.2,198.6).curveTo(-145.2,163.2,-168.6,93.2).closePath();
	var mask_graphics_296 = new cjs.Graphics().moveTo(-169.2,92.6).curveTo(-192.5,22.4,-162.5,-44.7).curveTo(-132.5,-111.9,-64.5,-141.2).curveTo(3.3,-170.5,73.1,-146.2).curveTo(142.8,-122,177.7,-56.7).curveTo(212.4,8.5,194.4,79.8).curveTo(176.3,151.2,113.9,191.7).lineTo(50.8,94.7).curveTo(75,78.8,82.2,51.1).curveTo(89.2,23.4,75.7,-2.1).curveTo(62,-27.4,34.9,-36.9).curveTo(7.7,-46.4,-18.6,-35).curveTo(-45.1,-23.5,-56.8,2.6).curveTo(-68.5,28.7,-59.3,56.1).curveTo(-50.3,83.4,-25.2,97.3).lineTo(-81.3,198.6).curveTo(-145.9,162.8,-169.2,92.6).closePath();
	var mask_graphics_297 = new cjs.Graphics().moveTo(-169.2,92.1).curveTo(-192.5,21.6,-161.8,-45.8).curveTo(-131.3,-113.1,-63.1,-141.8).curveTo(5.1,-170.6,74.7,-145.7).curveTo(144.3,-120.7,178.7,-54.8).curveTo(212.8,11.2,193.6,82.6).curveTo(174.3,154,111,193.5).lineTo(49.8,95.3).curveTo(74.3,80,81.9,52.2).curveTo(89.4,24.4,76.1,-1.3).curveTo(62.6,-27,35.6,-36.7).curveTo(8.5,-46.4,-17.9,-35.2).curveTo(-44.6,-24,-56.5,2.2).curveTo(-68.4,28.4,-59.3,55.9).curveTo(-50.3,83.4,-25.1,97.3).lineTo(-81.3,198.6).curveTo(-145.9,162.8,-169.2,92.1).closePath();
	var mask_graphics_298 = new cjs.Graphics().moveTo(-169.1,91.7).curveTo(-192.4,20.7,-161.5,-46.5).curveTo(-130.6,-113.8,-61.8,-142.2).curveTo(6.9,-170.6,76.7,-144.8).curveTo(146.3,-119,179.8,-52.6).curveTo(213.2,13.9,192.7,85.3).curveTo(172.1,156.8,108.3,195.2).lineTo(48.6,96.1).curveTo(73.5,81.1,81.5,53.2).curveTo(89.5,25.4,76.6,-0.5).curveTo(63.5,-26.3,36.4,-36.4).curveTo(9.1,-46.4,-17.6,-35.3).curveTo(-44.3,-24.3,-56.3,1.9).curveTo(-68.5,28.1,-59.4,55.7).curveTo(-50.4,83.3,-25.1,97.3).lineTo(-81.2,198.6).curveTo(-145.9,162.8,-169.1,91.7).closePath();
	var mask_graphics_299 = new cjs.Graphics().moveTo(-169.7,91.4).curveTo(-192.9,20.4,-161.3,-47.3).curveTo(-129.8,-115,-60.5,-143).curveTo(8.7,-171,78.5,-144.3).curveTo(148.1,-117.5,180.9,-50.4).curveTo(213.7,16.7,192,88.1).curveTo(170,159.5,105.3,196.9).lineTo(47.4,96.7).curveTo(72.6,82.2,81.1,54.4).curveTo(89.5,26.5,76.9,0.4).curveTo(64.2,-25.8,37.1,-36.1).curveTo(9.9,-46.5,-17.1,-35.6).curveTo(-44.1,-24.8,-56.4,1.6).curveTo(-68.7,27.9,-59.7,55.5).curveTo(-50.7,83.2,-25.3,97.3).lineTo(-81.4,198.6).curveTo(-146.6,162.4,-169.7,91.4).closePath();
	var mask_graphics_300 = new cjs.Graphics().moveTo(-169.7,91.4).curveTo(-192.9,20.4,-161.3,-47.3).curveTo(-129.8,-115,-60.5,-143).curveTo(8.7,-171,78.5,-144.3).curveTo(148.1,-117.5,180.9,-50.4).curveTo(213.7,16.7,192,88.1).curveTo(170,159.5,105.3,196.9).lineTo(47.4,96.7).curveTo(72.6,82.2,81.1,54.4).curveTo(89.5,26.5,76.9,0.4).curveTo(64.2,-25.8,37.1,-36.1).curveTo(9.9,-46.5,-17.1,-35.6).curveTo(-44.1,-24.8,-56.4,1.6).curveTo(-68.7,27.9,-59.7,55.5).curveTo(-50.7,83.2,-25.3,97.3).lineTo(-81.4,198.6).curveTo(-146.6,162.4,-169.7,91.4).closePath();
	var mask_graphics_301 = new cjs.Graphics().moveTo(-169.1,91.7).curveTo(-192.4,20.7,-161.5,-46.5).curveTo(-130.6,-113.8,-61.8,-142.2).curveTo(6.9,-170.6,76.7,-144.8).curveTo(146.3,-119,179.8,-52.6).curveTo(213.2,13.9,192.7,85.3).curveTo(172.1,156.8,108.3,195.2).lineTo(48.6,96.1).curveTo(73.5,81.1,81.5,53.2).curveTo(89.5,25.4,76.6,-0.5).curveTo(63.5,-26.3,36.4,-36.4).curveTo(9.1,-46.4,-17.6,-35.3).curveTo(-44.3,-24.3,-56.3,1.9).curveTo(-68.5,28.1,-59.4,55.7).curveTo(-50.4,83.3,-25.1,97.3).lineTo(-81.2,198.6).curveTo(-145.9,162.8,-169.1,91.7).closePath();
	var mask_graphics_302 = new cjs.Graphics().moveTo(-169.2,92.1).curveTo(-192.5,21.6,-161.8,-45.8).curveTo(-131.3,-113.1,-63.1,-141.8).curveTo(5.1,-170.6,74.7,-145.7).curveTo(144.3,-120.7,178.7,-54.8).curveTo(212.8,11.2,193.6,82.6).curveTo(174.3,154,111,193.5).lineTo(49.8,95.3).curveTo(74.3,80,81.9,52.2).curveTo(89.4,24.4,76.1,-1.3).curveTo(62.6,-27,35.6,-36.7).curveTo(8.5,-46.4,-17.9,-35.2).curveTo(-44.6,-24,-56.5,2.2).curveTo(-68.4,28.4,-59.3,55.9).curveTo(-50.3,83.4,-25.1,97.3).lineTo(-81.3,198.6).curveTo(-145.9,162.8,-169.2,92.1).closePath();
	var mask_graphics_303 = new cjs.Graphics().moveTo(-169.2,92.6).curveTo(-192.5,22.4,-162.5,-44.7).curveTo(-132.5,-111.9,-64.5,-141.2).curveTo(3.3,-170.5,73.1,-146.2).curveTo(142.8,-122,177.7,-56.7).curveTo(212.4,8.5,194.4,79.8).curveTo(176.3,151.2,113.9,191.7).lineTo(50.8,94.7).curveTo(75,78.8,82.2,51.1).curveTo(89.2,23.4,75.7,-2.1).curveTo(62,-27.4,34.9,-36.9).curveTo(7.7,-46.4,-18.6,-35).curveTo(-45.1,-23.5,-56.8,2.6).curveTo(-68.5,28.7,-59.3,56.1).curveTo(-50.3,83.4,-25.2,97.3).lineTo(-81.3,198.6).curveTo(-145.9,162.8,-169.2,92.6).closePath();
	var mask_graphics_304 = new cjs.Graphics().moveTo(-168.6,93.2).curveTo(-192.3,23.3,-162.5,-43.9).curveTo(-132.9,-110.9,-65.6,-140.5).curveTo(1.6,-170.1,71.2,-146.9).curveTo(140.8,-123.5,176.5,-58.8).curveTo(212,5.9,194.9,77.2).curveTo(177.7,148.7,116.7,189.9).lineTo(52.1,93.9).curveTo(75.8,78,82.5,50.1).curveTo(89.1,22.3,75.3,-2.8).curveTo(61.4,-28,34.3,-37.2).curveTo(7.1,-46.3,-18.9,-34.7).curveTo(-45.2,-23.1,-56.7,3).curveTo(-68.2,29.1,-59,56.4).curveTo(-49.9,83.5,-25.1,97.3).lineTo(-81.2,198.6).curveTo(-145.2,163.2,-168.6,93.2).closePath();
	var mask_graphics_305 = new cjs.Graphics().moveTo(-168.6,93.7).curveTo(-192.4,24.2,-163.3,-42.9).curveTo(-134.2,-109.9,-67.2,-139.9).curveTo(-0.2,-170,69.6,-147.3).curveTo(139.2,-124.7,175.5,-60.7).curveTo(211.6,3.2,195.6,74.4).curveTo(179.6,145.8,119.3,188.1).lineTo(53,93.2).curveTo(76.4,76.8,82.6,49).curveTo(88.8,21.3,74.8,-3.7).curveTo(60.7,-28.5,33.6,-37.4).curveTo(6.4,-46.2,-19.6,-34.5).curveTo(-45.7,-22.8,-57,3.4).curveTo(-68.3,29.4,-59.2,56.5).curveTo(-50,83.5,-25.1,97.3).lineTo(-81.2,198.6).curveTo(-145.1,163.2,-168.6,93.7).closePath();
	var mask_graphics_306 = new cjs.Graphics().moveTo(-168.7,94.1).curveTo(-192.4,25,-163.6,-42.1).curveTo(-134.8,-109.1,-68.4,-139.5).curveTo(-2,-169.9,67.6,-148.1).curveTo(137.2,-126.2,174.1,-62.9).curveTo(211,0.5,196.4,71.6).curveTo(181.5,142.8,122.1,186.2).lineTo(54,92.4).curveTo(77.1,75.6,82.9,47.9).curveTo(88.6,20.3,74.2,-4.4).curveTo(59.9,-29.1,32.9,-37.6).curveTo(5.7,-46.1,-20.1,-34.2).curveTo(-46,-22.4,-57.1,3.6).curveTo(-68.4,29.7,-59.1,56.7).curveTo(-50,83.5,-25.2,97.3).lineTo(-81.3,198.6).curveTo(-145.1,163.2,-168.7,94.1).closePath();
	var mask_graphics_307 = new cjs.Graphics().moveTo(-168.5,94.5).curveTo(-192.3,25.7,-163.8,-41.3).curveTo(-135.5,-108.3,-69.6,-139.1).curveTo(-3.6,-169.9,66.1,-148.6).curveTo(135.7,-127.2,173.2,-65).curveTo(210.6,-2.6,196.8,68.9).curveTo(182.9,140.3,124.8,184.1).lineTo(55,91.7).curveTo(77.6,74.7,83.1,46.9).curveTo(88.5,19.1,73.9,-5.2).curveTo(59.3,-29.6,32.3,-37.9).curveTo(5.1,-46.1,-20.5,-34.2).curveTo(-46.3,-22.1,-57.3,3.9).curveTo(-68.4,30,-59.1,56.8).curveTo(-49.9,83.6,-25.1,97.3).lineTo(-81.3,198.6).curveTo(-144.9,163.3,-168.5,94.5).closePath();
	var mask_graphics_308 = new cjs.Graphics().moveTo(-168,94.9).curveTo(-191.9,26.1,-164.1,-40.4).curveTo(-136.3,-106.9,-70.9,-138.2).curveTo(-5.4,-169.4,64.1,-149.1).curveTo(133.6,-128.7,171.8,-67).curveTo(209.9,-5.3,197.3,66).curveTo(184.7,137.4,127.5,182.2).lineTo(56.2,90.9).curveTo(78.4,73.5,83.4,45.8).curveTo(88.3,18,73.4,-6).curveTo(58.6,-30.1,31.6,-38.1).curveTo(4.5,-45.9,-21,-33.7).curveTo(-46.6,-21.6,-57.3,4.2).curveTo(-68.1,30.1,-58.8,56.9).curveTo(-49.6,83.7,-25,97.3).lineTo(-81.2,198.6).curveTo(-144.2,163.7,-168,94.9).closePath();
	var mask_graphics_309 = new cjs.Graphics().moveTo(-168.1,95.3).curveTo(-192,27,-164.5,-39.7).curveTo(-137.1,-106.2,-72.1,-137.7).curveTo(-7.2,-169.1,62.2,-149.7).curveTo(131.5,-130.3,170.4,-69.1).curveTo(209.3,-8,197.9,63.1).curveTo(186.5,134.3,130,180).lineTo(57.1,90.1).curveTo(79,72.3,83.5,44.6).curveTo(87.9,16.9,72.9,-6.9).curveTo(57.7,-30.6,30.8,-38.3).curveTo(3.8,-45.8,-21.4,-33.6).curveTo(-46.8,-21.4,-57.5,4.6).curveTo(-68.3,30.5,-58.9,57.1).curveTo(-49.6,83.7,-25.1,97.3).lineTo(-81.2,198.6).curveTo(-144.2,163.7,-168.1,95.3).closePath();
	var mask_graphics_310 = new cjs.Graphics().moveTo(-168,95.7).curveTo(-192,27.8,-165.1,-38.6).curveTo(-138.2,-105,-73.6,-137).curveTo(-9,-169,60.5,-150.2).curveTo(129.9,-131.4,169.3,-71).curveTo(208.6,-10.7,198.4,60.3).curveTo(188.2,131.3,132.5,178).lineTo(58.2,89.3).curveTo(79.8,71.2,83.8,43.6).curveTo(87.7,15.8,72.4,-7.7).curveTo(57.1,-31.2,30.1,-38.5).curveTo(3,-45.8,-22.1,-33.4).curveTo(-47.3,-20.8,-57.7,5).curveTo(-68.3,30.8,-58.9,57.2).curveTo(-49.6,83.7,-25.1,97.3).lineTo(-81.2,198.6).curveTo(-144.1,163.7,-168,95.7).closePath();
	var mask_graphics_311 = new cjs.Graphics().moveTo(-167.5,96.4).curveTo(-191.6,28.7,-165,-37.7).curveTo(-138.4,-104.1,-74.4,-136.3).curveTo(-10.6,-168.5,58.7,-150.5).curveTo(127.9,-132.6,168,-72.9).curveTo(208,-13.1,198.5,58.1).curveTo(189.1,129.4,135.3,175.9).lineTo(59.3,88.4).curveTo(80.3,70.3,84,42.7).curveTo(87.6,14.9,72,-8.4).curveTo(56.4,-31.6,29.5,-38.6).curveTo(2.5,-45.5,-22.4,-33).curveTo(-47.3,-20.5,-57.6,5.4).curveTo(-68,31.2,-58.6,57.5).curveTo(-49.2,83.9,-24.9,97.3).lineTo(-81.1,198.6).curveTo(-143.4,164.1,-167.5,96.4).closePath();
	var mask_graphics_312 = new cjs.Graphics().moveTo(-167.4,96.7).curveTo(-191.6,29.5,-165.6,-36.8).curveTo(-139.6,-102.9,-76,-135.6).curveTo(-12.4,-168.3,57,-151.1).curveTo(126.3,-133.8,166.8,-74.8).curveTo(207.2,-15.8,199,55.2).curveTo(190.6,126.2,137.7,173.7).lineTo(60.1,87.5).curveTo(80.8,69.1,84,41.4).curveTo(87.2,13.8,71.5,-9.2).curveTo(55.7,-32.1,28.8,-38.8).curveTo(1.8,-45.4,-22.9,-32.8).curveTo(-47.7,-20.1,-57.9,5.7).curveTo(-68.1,31.5,-58.6,57.6).curveTo(-49.2,83.9,-25,97.3).lineTo(-81.1,198.6).curveTo(-143.3,164.1,-167.4,96.7).closePath();
	var mask_graphics_313 = new cjs.Graphics().moveTo(-167.4,97.2).curveTo(-191.7,30.4,-165.9,-35.9).curveTo(-140.3,-102.2,-77.2,-135.2).curveTo(-14.2,-168.1,55,-151.6).curveTo(124.1,-135.1,165.3,-76.8).curveTo(206.5,-18.4,199.3,52.3).curveTo(192.2,123.1,140,171.4).lineTo(61.1,86.7).curveTo(81.4,67.9,84.3,40.4).curveTo(86.9,12.9,70.9,-9.9).curveTo(54.9,-32.6,28,-38.9).curveTo(1.1,-45.4,-23.5,-32.6).curveTo(-48.1,-19.8,-58,6).curveTo(-68,31.9,-58.6,57.8).curveTo(-49.2,83.9,-25,97.3).lineTo(-81.1,198.6).curveTo(-143.3,164.1,-167.4,97.2).closePath();
	var mask_graphics_314 = new cjs.Graphics().moveTo(-167.3,97.6).curveTo(-191.7,31,-166.2,-35.2).curveTo(-140.8,-101.4,-78.3,-134.7).curveTo(-15.7,-168,53.3,-152).curveTo(122.4,-136,164.1,-78.7).curveTo(205.8,-21.4,199.6,49.2).curveTo(193.4,119.9,142.5,169.1).lineTo(62,85.8).curveTo(81.9,66.6,84.3,39.2).curveTo(86.7,11.7,70.5,-10.6).curveTo(54.2,-32.9,27.4,-39.1).curveTo(0.5,-45.4,-23.9,-32.4).curveTo(-48.3,-19.4,-58.1,6.4).curveTo(-68,32.2,-58.5,58).curveTo(-49.2,83.9,-25,97.3).lineTo(-81.2,198.6).curveTo(-143.1,164.3,-167.3,97.6).closePath();
	var mask_graphics_315 = new cjs.Graphics().moveTo(-166.7,98).curveTo(-191.1,31.4,-166.4,-34.3).curveTo(-141.7,-100.1,-79.5,-133.8).curveTo(-17.4,-167.3,51.5,-152.3).curveTo(120.4,-137.4,162.6,-80.9).curveTo(204.7,-24.2,199.7,46.5).curveTo(194.7,117.3,145,166.8).lineTo(63.1,84.9).curveTo(82.5,65.6,84.4,38.1).curveTo(86.3,10.6,69.9,-11.5).curveTo(53.5,-33.4,26.7,-39.3).curveTo(-0.2,-45,-24.3,-31.9).curveTo(-48.6,-18.9,-58.1,6.6).curveTo(-67.8,32.2,-58.3,58.1).curveTo(-48.8,84.1,-24.9,97.3).lineTo(-81,198.6).curveTo(-142.4,164.6,-166.7,98).closePath();
	var mask_graphics_316 = new cjs.Graphics().moveTo(-166.8,98.5).curveTo(-191.2,32.3,-166.7,-33.6).curveTo(-142.3,-99.3,-80.8,-133.2).curveTo(-19.3,-167.1,49.5,-153).curveTo(118.2,-138.7,161.1,-82.8).curveTo(203.9,-26.7,200,43.8).curveTo(196.1,114.2,147.2,164.4).lineTo(63.9,84).curveTo(82.9,64.5,84.5,37).curveTo(86.1,9.6,69.4,-12.2).curveTo(52.6,-34,26,-39.5).curveTo(-0.9,-45,-24.7,-31.8).curveTo(-48.7,-18.7,-58.3,6.9).curveTo(-67.8,32.5,-58.2,58.3).curveTo(-48.9,84.1,-24.9,97.3).lineTo(-81,198.6).curveTo(-142.4,164.6,-166.8,98.5).closePath();
	var mask_graphics_317 = new cjs.Graphics().moveTo(-166.8,98.8).curveTo(-191.3,33.1,-167.4,-32.5).curveTo(-143.4,-98,-82.3,-132.4).curveTo(-21.2,-166.7,47.7,-153.1).curveTo(116.5,-139.6,159.8,-84.5).curveTo(203,-29.3,200.2,40.9).curveTo(197.3,110.9,149.3,162.1).lineTo(64.7,83.1).curveTo(83.3,63.2,84.6,35.9).curveTo(85.6,8.6,68.9,-12.9).curveTo(52,-34.3,25.1,-39.6).curveTo(-1.7,-44.9,-25.5,-31.6).curveTo(-49.3,-18.1,-58.6,7.3).curveTo(-67.9,32.8,-58.3,58.5).curveTo(-48.8,84.1,-25,97.3).lineTo(-81.1,198.6).curveTo(-142.4,164.7,-166.8,98.8).closePath();
	var mask_graphics_318 = new cjs.Graphics().moveTo(-166.2,99.5).curveTo(-190.9,34,-167.2,-31.6).curveTo(-143.8,-97.1,-83.2,-131.6).curveTo(-22.7,-166.2,45.8,-153.5).curveTo(114.3,-140.8,158.3,-86.2).curveTo(202.1,-31.7,200.2,38.2).curveTo(198.2,108.2,151.8,159.6).lineTo(65.7,82.2).curveTo(83.8,62.2,84.6,34.9).curveTo(85.4,7.6,68.3,-13.6).curveTo(51.2,-34.9,24.5,-39.8).curveTo(-2.2,-44.6,-25.8,-31.2).curveTo(-49.3,-17.8,-58.5,7.6).curveTo(-67.7,33.2,-58,58.7).curveTo(-48.4,84.3,-24.9,97.3).lineTo(-81,198.6).curveTo(-141.6,165,-166.2,99.5).closePath();
	var mask_graphics_319 = new cjs.Graphics().moveTo(-166.2,99.9).curveTo(-190.9,34.8,-167.9,-30.5).curveTo(-144.9,-95.8,-84.6,-130.8).curveTo(-24.5,-165.8,44.1,-153.8).curveTo(112.6,-141.7,156.9,-88).curveTo(201.1,-34.3,200.3,35.3).curveTo(199.5,105,153.9,157.1).lineTo(66.5,81.1).curveTo(84.2,60.9,84.6,33.7).curveTo(84.9,6.6,67.7,-14.3).curveTo(50.5,-35.2,23.8,-39.9).curveTo(-3,-44.6,-26.4,-30.9).curveTo(-49.9,-17.3,-58.7,8.2).curveTo(-67.6,33.6,-58,58.9).curveTo(-48.5,84.3,-25,97.3).lineTo(-81.1,198.6).curveTo(-141.6,165.1,-166.2,99.9).closePath();
	var mask_graphics_320 = new cjs.Graphics().moveTo(-166.3,100.3).curveTo(-191,35.6,-168.3,-29.7).curveTo(-145.6,-95,-85.9,-130.3).curveTo(-26.3,-165.5,42.1,-154.2).curveTo(110.3,-143,155.2,-90).curveTo(200.1,-36.8,200.4,32.4).curveTo(200.6,101.7,156,154.6).lineTo(67.3,80.2).curveTo(84.6,59.6,84.5,32.6).curveTo(84.4,5.6,67,-15).curveTo(49.5,-35.7,22.9,-40.1).curveTo(-3.6,-44.4,-26.8,-30.8).curveTo(-50.1,-17,-58.9,8.4).curveTo(-67.7,33.9,-58.1,59).curveTo(-48.6,84.3,-25,97.3).lineTo(-81.2,198.6).curveTo(-141.7,165.1,-166.3,100.3).closePath();
	var mask_graphics_321 = new cjs.Graphics().moveTo(-166.2,100.7).curveTo(-190.9,36.3,-168.4,-28.9).curveTo(-146,-94.2,-86.9,-129.7).curveTo(-27.8,-165.3,40.4,-154.5).curveTo(108.5,-143.7,153.9,-91.7).curveTo(199.1,-39.5,200.4,29.5).curveTo(201.5,98.4,158.1,152.1).lineTo(68.1,79.3).curveTo(85,58.3,84.6,31.4).curveTo(84.1,4.7,66.5,-15.6).curveTo(48.8,-35.9,22.3,-40.1).curveTo(-4.3,-44.3,-27.2,-30.5).curveTo(-50.3,-16.7,-58.9,8.7).curveTo(-67.7,34.2,-58.1,59.2).curveTo(-48.5,84.3,-25,97.3).lineTo(-81.1,198.6).curveTo(-141.5,165.1,-166.2,100.7).closePath();
	var mask_graphics_322 = new cjs.Graphics().moveTo(-165.6,101.1).curveTo(-190.5,36.7,-168.6,-28.1).curveTo(-146.8,-92.8,-88.1,-128.7).curveTo(-29.5,-164.5,38.8,-154.6).curveTo(107,-144.6,152.5,-93.4).curveTo(197.8,-42.3,200.2,26.7).curveTo(202.3,95.7,160.3,149.5).lineTo(69,78.1).curveTo(85.4,57.2,84.5,30.4).curveTo(83.6,3.5,66,-16.4).curveTo(48.3,-36.3,21.7,-40.2).curveTo(-4.9,-44.1,-27.7,-30.1).curveTo(-50.6,-16.1,-59,9.1).curveTo(-67.5,34.2,-57.7,59.3).curveTo(-48.1,84.5,-24.9,97.3).lineTo(-81,198.6).curveTo(-140.7,165.5,-165.6,101.1).closePath();
	var mask_graphics_323 = new cjs.Graphics().moveTo(-165.6,101.4).curveTo(-190.5,37.4,-168.9,-27.3).curveTo(-147.4,-92.1,-89.3,-128.2).curveTo(-31.3,-164.1,36.7,-155).curveTo(104.7,-145.8,150.7,-95.3).curveTo(196.7,-44.8,200,23.8).curveTo(203.4,92.4,162.2,146.8).lineTo(69.8,77.2).curveTo(85.8,56,84.5,29.3).curveTo(83.2,2.6,65.3,-17.1).curveTo(47.4,-36.8,21,-40.3).curveTo(-5.5,-43.8,-28.1,-29.9).curveTo(-50.8,-15.8,-59.2,9.4).curveTo(-67.6,34.7,-57.9,59.6).curveTo(-48.1,84.5,-24.9,97.3).lineTo(-81,198.6).curveTo(-140.7,165.5,-165.6,101.4).closePath();
	var mask_graphics_324 = new cjs.Graphics().moveTo(-165.5,102).curveTo(-190.5,38.4,-169.4,-26.2).curveTo(-148.5,-90.8,-90.8,-127.3).curveTo(-33.1,-163.7,34.9,-155.1).curveTo(102.9,-146.6,149.2,-96.9).curveTo(195.5,-47.2,199.9,20.8).curveTo(204.3,89,164.2,144.3).lineTo(70.5,76.1).curveTo(86.1,54.7,84.5,28.1).curveTo(82.7,1.6,64.8,-17.7).curveTo(46.7,-37,20.2,-40.4).curveTo(-6.3,-43.7,-28.7,-29.5).curveTo(-51.2,-15.4,-59.3,9.9).curveTo(-67.6,35,-57.9,59.7).curveTo(-48.2,84.5,-25,97.3).lineTo(-81.1,198.6).curveTo(-140.7,165.6,-165.5,102).closePath();
	var mask_graphics_325 = new cjs.Graphics().moveTo(-165,102.5).curveTo(-190.1,39.2,-169.4,-25.3).curveTo(-148.8,-89.8,-91.7,-126.4).curveTo(-34.7,-163.1,33,-155.3).curveTo(100.7,-147.5,147.5,-98.5).curveTo(194.4,-49.5,199.7,18.3).curveTo(204.9,86.3,166.1,141.4).lineTo(71.3,75.1).curveTo(86.3,53.5,84.4,27.1).curveTo(82.3,0.7,64.1,-18.4).curveTo(45.8,-37.3,19.5,-40.5).curveTo(-6.8,-43.4,-29,-29.2).curveTo(-51.3,-15,-59.3,10.2).curveTo(-67.4,35.2,-57.6,59.9).curveTo(-47.9,84.6,-24.9,97.3).lineTo(-81,198.6).curveTo(-140,165.9,-165,102.5).closePath();
	var mask_graphics_326 = new cjs.Graphics().moveTo(-165,103).curveTo(-190.1,40,-170,-24.2).curveTo(-149.9,-88.5,-93.1,-125.6).curveTo(-36.5,-162.7,31.2,-155.5).curveTo(98.9,-148.3,146.1,-100.2).curveTo(193.2,-51.9,199.5,15.5).curveTo(205.6,83,168,138.8).lineTo(72,74.1).curveTo(86.6,52.3,84.2,26.1).curveTo(81.7,-0.2,63.5,-19).curveTo(45.1,-37.7,18.8,-40.5).curveTo(-7.6,-43.4,-29.6,-29).curveTo(-51.8,-14.4,-59.5,10.5).curveTo(-67.4,35.6,-57.6,60.1).curveTo(-48,84.6,-25,97.3).lineTo(-81.1,198.6).curveTo(-140,166,-165,103).closePath();
	var mask_graphics_327 = new cjs.Graphics().moveTo(-165.1,103.4).curveTo(-190.2,40.9,-170.4,-23.5).curveTo(-150.5,-87.7,-94.3,-125).curveTo(-38.2,-162.2,29.2,-155.8).curveTo(96.5,-149.4,144.2,-102).curveTo(191.9,-54.4,199.2,12.6).curveTo(206.5,79.6,169.8,136.1).lineTo(72.7,73).curveTo(86.9,51,84.2,24.8).curveTo(81.3,-1.2,62.7,-19.7).curveTo(44.2,-38.2,18,-40.6).curveTo(-8.3,-43.1,-30.2,-28.6).curveTo(-52,-14.1,-59.7,10.9).curveTo(-67.5,35.9,-57.6,60.2).curveTo(-47.9,84.7,-25,97.3).lineTo(-81.1,198.6).curveTo(-140,166,-165.1,103.4).closePath();
	var mask_graphics_328 = new cjs.Graphics().moveTo(-165.1,103.8).curveTo(-190.2,41.6,-170.5,-22.6).curveTo(-150.9,-86.8,-95.3,-124.4).curveTo(-39.7,-161.9,27.3,-156.1).curveTo(94.1,-150.3,142.4,-103.7).curveTo(190.7,-57,198.9,9.6).curveTo(207.1,76.2,171.5,133.2).lineTo(73.2,71.9).curveTo(87.1,49.7,83.9,23.8).curveTo(80.7,-2.2,61.9,-20.3).curveTo(43.2,-38.4,17.2,-40.7).curveTo(-8.9,-43,-30.5,-28.3).curveTo(-52.2,-13.7,-59.8,11.2).curveTo(-67.5,36.2,-57.7,60.4).curveTo(-47.9,84.7,-25.1,97.3).lineTo(-81.2,198.6).curveTo(-139.9,166,-165.1,103.8).closePath();
	var mask_graphics_329 = new cjs.Graphics().moveTo(-164.4,104.1).curveTo(-189.8,41.9,-170.7,-21.8).curveTo(-151.7,-85.5,-96.5,-123.3).curveTo(-41.5,-161.1,25.5,-156).curveTo(92.5,-150.9,140.9,-105.3).curveTo(189.1,-59.6,198.4,7).curveTo(207.5,73.5,173.3,130.4).lineTo(74.1,70.8).curveTo(87.4,48.6,83.9,22.7).curveTo(80.3,-3.2,61.6,-21).curveTo(42.6,-38.8,16.6,-40.8).curveTo(-9.5,-42.8,-31,-28).curveTo(-52.5,-13.3,-59.9,11.5).curveTo(-67.3,36.4,-57.4,60.6).curveTo(-47.6,84.8,-24.9,97.3).lineTo(-81,198.6).curveTo(-139.2,166.4,-164.4,104.1).closePath();
	var mask_graphics_330 = new cjs.Graphics().moveTo(-164.3,104.6).curveTo(-189.7,42.8,-170.9,-21).curveTo(-152.3,-84.7,-97.7,-122.7).curveTo(-43.2,-160.6,23.5,-156.4).curveTo(90.1,-152,139,-107).curveTo(187.8,-62,198,4).curveTo(208.1,70,175,127.5).lineTo(74.7,69.7).curveTo(87.6,47.3,83.7,21.6).curveTo(79.7,-4.1,60.7,-21.6).curveTo(41.7,-39.1,15.8,-40.8).curveTo(-10.2,-42.5,-31.4,-27.8).curveTo(-52.7,-13,-60,11.8).curveTo(-67.3,36.7,-57.4,60.8).curveTo(-47.6,84.8,-25,97.3).lineTo(-81.1,198.6).curveTo(-139.1,166.4,-164.3,104.6).closePath();
	var mask_graphics_331 = new cjs.Graphics().moveTo(-171.7,85.1).curveTo(-193.4,9.6,-155.3,-59.1).curveTo(-117.2,-127.8,-41.8,-149.5).curveTo(33.7,-171.1,102.4,-133.1).curveTo(171.1,-95,192.8,-19.5).curveTo(214.4,56,176.4,124.7).lineTo(75.1,68.5).curveTo(89.9,41.8,81.6,12.4).curveTo(73.1,-17,46.3,-31.8).curveTo(19.5,-46.6,-9.7,-38.2).curveTo(-39.2,-29.8,-54,-3).curveTo(-68.9,23.8,-60.3,53.1).curveTo(-52,82.5,-25.2,97.3).lineTo(-81.4,198.6).curveTo(-150.1,160.5,-171.7,85.1).closePath();
	var mask_graphics_332 = new cjs.Graphics().moveTo(-171.2,85.7).curveTo(-193.1,10.6,-155.7,-58).curveTo(-118.3,-126.6,-43.2,-148.9).curveTo(31.8,-171.2,100.4,-134.2).curveTo(168.9,-97.3,191.8,-22.1).curveTo(214.6,53,178,121.8).lineTo(75.8,67.4).curveTo(90,40.7,81.2,11.3).curveTo(72.3,-17.9,45.5,-32.2).curveTo(18.8,-46.6,-10.3,-37.9).curveTo(-39.6,-29.2,-54,-2.6).curveTo(-68.7,24.1,-60.1,53.3).curveTo(-51.7,82.6,-25.2,97.3).lineTo(-81.3,198.6).curveTo(-149.4,160.9,-171.2,85.7).closePath();
	var mask_graphics_333 = new cjs.Graphics().moveTo(-171.2,86.2).curveTo(-193.3,11.5,-156.4,-57).curveTo(-119.6,-125.4,-44.8,-148.4).curveTo(29.9,-171.2,98.6,-135.3).curveTo(167.1,-99.4,191,-25).curveTo(214.8,49.5,179.5,118.8).lineTo(76.4,66.3).curveTo(90,39.3,80.9,10.3).curveTo(71.6,-18.7,44.9,-32.8).curveTo(18,-46.7,-11,-37.9).curveTo(-40.1,-28.9,-54.4,-2.3).curveTo(-68.7,24.4,-60.1,53.5).curveTo(-51.6,82.7,-25.2,97.3).lineTo(-81.3,198.6).curveTo(-149.3,160.9,-171.2,86.2).closePath();
	var mask_graphics_334 = new cjs.Graphics().moveTo(-171.2,86.6).curveTo(-193.3,12.2,-156.9,-56.3).curveTo(-120.5,-124.7,-46.4,-148).curveTo(27.5,-171.3,96.7,-136.1).curveTo(165.7,-100.9,190.3,-27.4).curveTo(214.9,46.2,180.9,115.9).lineTo(76.9,65).curveTo(90.1,38,80.6,9.4).curveTo(70.9,-19.2,44,-33).curveTo(17.1,-46.7,-11.7,-37.6).curveTo(-40.5,-28.5,-54.6,-1.9).curveTo(-68.8,24.7,-60.2,53.7).curveTo(-51.7,82.7,-25.2,97.3).lineTo(-81.4,198.6).curveTo(-149.2,161,-171.2,86.6).closePath();
	var mask_graphics_335 = new cjs.Graphics().moveTo(-170.7,87.3).curveTo(-193,13.3,-157.2,-55.1).curveTo(-121.5,-123.4,-48,-147.4).curveTo(25.5,-171.2,94.5,-137.3).curveTo(163.4,-103.3,189.2,-30).curveTo(214.9,43.2,182.5,112.8).lineTo(77.5,64).curveTo(90.1,36.9,80.2,8.3).curveTo(70.1,-20.2,43.3,-33.5).curveTo(16.4,-46.7,-12.1,-37.4).curveTo(-40.8,-28.1,-54.7,-1.4).curveTo(-68.7,25.2,-60,54).curveTo(-51.4,82.8,-25.2,97.3).lineTo(-81.3,198.6).curveTo(-148.6,161.3,-170.7,87.3).closePath();
	var mask_graphics_336 = new cjs.Graphics().moveTo(-170.7,87.8).curveTo(-193.1,14.2,-157.9,-54.1).curveTo(-122.9,-122.3,-49.5,-146.8).curveTo(23.6,-171.3,92.7,-138.3).curveTo(161.5,-105.3,188.3,-32.9).curveTo(215,39.7,183.8,109.9).lineTo(78,62.7).curveTo(90.2,35.5,79.8,7.3).curveTo(69.3,-20.9,42.5,-33.8).curveTo(15.6,-46.7,-12.8,-37.1).curveTo(-41.3,-27.6,-54.9,-1).curveTo(-68.7,25.5,-60,54.2).curveTo(-51.4,82.8,-25.2,97.3).lineTo(-81.3,198.6).curveTo(-148.5,161.4,-170.7,87.8).closePath();
	var mask_graphics_337 = new cjs.Graphics().moveTo(-170.7,88.5).curveTo(-193.1,15.2,-158.4,-53).curveTo(-123.7,-121.2,-51.2,-146.2).curveTo(21.2,-171.2,90.7,-138.8).curveTo(160.1,-106.4,187.5,-35).curveTo(214.9,36.5,185,107.1).lineTo(78.5,61.8).curveTo(90.1,34.4,79.5,6.5).curveTo(68.8,-21.3,41.7,-33.9).curveTo(14.6,-46.5,-13.5,-36.8).curveTo(-41.7,-27,-55.2,-0.4).curveTo(-68.8,26.1,-60,54.6).curveTo(-51.3,83.1,-25.3,97.6).lineTo(-81.4,198.8).curveTo(-148.4,161.7,-170.7,88.5).closePath();
	var mask_graphics_338 = new cjs.Graphics().moveTo(-170.2,89.2).curveTo(-192.9,16.3,-158.7,-51.8).curveTo(-124.7,-119.9,-52.6,-145.5).curveTo(19.3,-171,88.6,-139.9).curveTo(157.7,-108.7,186.3,-37.6).curveTo(214.8,33.6,186.4,104).lineTo(79,60.7).curveTo(90,33.2,78.9,5.5).curveTo(67.8,-22.1,40.9,-34.3).curveTo(14,-46.4,-13.9,-36.5).curveTo(-42.1,-26.5,-55.3,-0.1).curveTo(-68.6,26.4,-59.7,54.9).curveTo(-51,83.3,-25.2,97.6).lineTo(-81.3,198.8).curveTo(-147.6,162.1,-170.2,89.2).closePath();
	var mask_graphics_339 = new cjs.Graphics().moveTo(-170.3,89.6).curveTo(-193,17.1,-159.4,-50.8).curveTo(-125.9,-118.8,-54.2,-144.8).curveTo(17.4,-171,86.6,-140.8).curveTo(155.8,-110.5,185.3,-40.3).curveTo(214.7,30,187.5,100.9).lineTo(79.5,59.5).curveTo(90,31.8,78.6,4.4).curveTo(67.1,-22.8,40.2,-34.6).curveTo(13.3,-46.5,-14.6,-36.4).curveTo(-42.6,-26.2,-55.6,0.4).curveTo(-68.7,26.9,-59.8,55).curveTo(-51,83.3,-25.2,97.6).lineTo(-81.3,198.8).curveTo(-147.7,162.1,-170.3,89.6).closePath();
	var mask_graphics_340 = new cjs.Graphics().moveTo(-170.3,90).curveTo(-193,17.9,-159.9,-50.1).curveTo(-126.7,-118.1,-55.8,-144.6).curveTo(14.9,-171.1,84.7,-141.6).curveTo(154.2,-112,184.4,-42.7).curveTo(214.5,26.7,188.7,97.8).lineTo(79.8,58.3).curveTo(89.9,30.6,78.3,3.5).curveTo(66.4,-23.5,39.4,-34.9).curveTo(12.3,-46.4,-15.3,-36.1).curveTo(-42.9,-25.9,-55.7,0.6).curveTo(-68.7,27.2,-59.8,55.2).curveTo(-50.9,83.3,-25.2,97.6).lineTo(-81.4,198.8).curveTo(-147.6,162.2,-170.3,90).closePath();
	var mask_graphics_341 = new cjs.Graphics().moveTo(-169.7,90.7).curveTo(-192.8,18.9,-160.2,-49).curveTo(-127.8,-116.7,-57.4,-143.8).curveTo(13,-170.8,82.5,-142.5).curveTo(151.8,-114.2,183,-45.1).curveTo(214.2,23.8,189.9,94.7).lineTo(80.5,57).curveTo(89.9,29.4,77.8,2.6).curveTo(65.6,-24.3,38.6,-35.4).curveTo(11.5,-46.3,-15.8,-35.8).curveTo(-43.3,-25.3,-55.9,1.1).curveTo(-68.6,27.5,-59.6,55.4).curveTo(-50.6,83.5,-25.2,97.6).lineTo(-81.3,198.8).curveTo(-146.8,162.5,-169.7,90.7).closePath();
	var mask_graphics_342 = new cjs.Graphics().moveTo(-169.8,91.1).curveTo(-192.8,19.9,-160.8,-47.9).curveTo(-128.9,-115.5,-58.9,-143.1).curveTo(11.1,-170.7,80.5,-143.4).curveTo(149.8,-116,181.9,-47.9).curveTo(214,20.3,190.9,91.6).lineTo(80.8,55.8).curveTo(89.8,28.1,77.4,1.6).curveTo(64.9,-25,37.8,-35.7).curveTo(10.8,-46.3,-16.4,-35.5).curveTo(-43.7,-24.8,-56.1,1.5).curveTo(-68.6,27.8,-59.5,55.6).curveTo(-50.6,83.5,-25.2,97.6).lineTo(-81.3,198.8).curveTo(-146.8,162.5,-169.8,91.1).closePath();
	var mask_graphics_343 = new cjs.Graphics().moveTo(-169.7,91.7).curveTo(-192.8,20.6,-161.2,-47).curveTo(-129.8,-114.7,-60.5,-142.7).curveTo(8.8,-170.8,78.5,-144.1).curveTo(148.1,-117.2,180.9,-50.1).curveTo(213.7,17,191.8,88.4).lineTo(81.1,54.6).curveTo(89.6,26.8,77,0.6).curveTo(64.2,-25.5,37.1,-35.9).curveTo(9.9,-46.3,-17,-35.4).curveTo(-44.1,-24.5,-56.4,1.8).curveTo(-68.6,28.1,-59.6,55.7).curveTo(-50.7,83.5,-25.2,97.6).lineTo(-81.3,198.8).curveTo(-146.6,162.7,-169.7,91.7).closePath();
	var mask_graphics_344 = new cjs.Graphics().moveTo(-169.2,92.4).curveTo(-192.5,21.7,-161.6,-45.9).curveTo(-130.7,-113.4,-62,-141.9).curveTo(6.8,-170.4,76.2,-144.9).curveTo(145.7,-119.3,179.5,-52.7).curveTo(213.2,14,192.9,85.2).lineTo(81.6,53.3).curveTo(89.5,25.6,76.4,-0.4).curveTo(63.2,-26.3,36.2,-36.3).curveTo(9.1,-46.1,-17.6,-35.1).curveTo(-44.3,-23.9,-56.4,2.2).curveTo(-68.5,28.5,-59.4,56).curveTo(-50.4,83.6,-25.1,97.6).lineTo(-81.2,198.8).curveTo(-145.9,163,-169.2,92.4).closePath();
	var mask_graphics_345 = new cjs.Graphics().moveTo(-169.2,92.7).curveTo(-192.5,22.5,-162.1,-44.9).curveTo(-132,-112.3,-63.4,-141.3).curveTo(5,-170.3,74.3,-145.6).curveTo(143.7,-121,178.3,-55.2).curveTo(212.8,10.5,193.7,82).lineTo(82,52.1).curveTo(89.3,24.3,76,-1.4).curveTo(62.5,-27,35.4,-36.5).curveTo(8.4,-46.1,-18.2,-34.8).curveTo(-44.8,-23.6,-56.6,2.7).curveTo(-68.4,29,-59.3,56.4).curveTo(-50.2,83.7,-25.1,97.6).lineTo(-81.2,198.8).curveTo(-145.9,163,-169.2,92.7).closePath();
	var mask_graphics_346 = new cjs.Graphics().moveTo(-169,93.2).curveTo(-192.5,23.3,-162.5,-44).curveTo(-132.6,-111.4,-64.9,-140.8).curveTo(2.6,-170.2,72.4,-146.3).curveTo(142,-122.2,177.2,-57.5).curveTo(212.3,7.3,194.5,78.9).lineTo(82.2,50.8).curveTo(89.2,23,75.5,-2.2).curveTo(61.8,-27.5,34.7,-36.8).curveTo(7.6,-46.1,-18.8,-34.7).curveTo(-45.1,-23.2,-56.7,3).curveTo(-68.4,29.3,-59.3,56.5).curveTo(-50.3,83.7,-25.1,97.6).lineTo(-81.3,198.8).curveTo(-145.7,163.1,-169,93.2).closePath();
	var mask_graphics_347 = new cjs.Graphics().moveTo(-168.6,93.9).curveTo(-192.3,24.3,-162.9,-42.9).curveTo(-133.6,-110.1,-66.5,-139.9).curveTo(0.6,-169.8,70.1,-147.1).curveTo(139.5,-124.3,175.6,-59.9).curveTo(211.7,4.3,195.4,75.6).lineTo(82.6,49.6).curveTo(89,21.9,75,-3.2).curveTo(60.8,-28.2,33.9,-37.1).curveTo(6.7,-45.9,-19.3,-34.4).curveTo(-45.5,-22.7,-56.9,3.5).curveTo(-68.3,29.7,-59.1,56.7).curveTo(-50,83.8,-25.1,97.6).lineTo(-81.2,198.8).curveTo(-145.1,163.5,-168.6,93.9).closePath();
	var mask_graphics_348 = new cjs.Graphics().moveTo(-168.7,94.3).curveTo(-192.3,25.2,-163.5,-41.8).curveTo(-134.7,-108.9,-67.9,-139.3).curveTo(-1.2,-169.7,68.2,-147.8).curveTo(137.3,-125.9,174.3,-62.5).curveTo(211.2,0.9,196.1,72.5).lineTo(82.8,48.4).curveTo(88.7,20.5,74.3,-4.2).curveTo(60,-28.8,33.1,-37.4).curveTo(6.1,-45.9,-19.8,-34).curveTo(-45.9,-22.2,-57.1,3.8).curveTo(-68.3,30,-59.1,56.9).curveTo(-50,83.8,-25.1,97.6).lineTo(-81.2,198.8).curveTo(-145.1,163.5,-168.7,94.3).closePath();
	var mask_graphics_349 = new cjs.Graphics().moveTo(-168.5,94.8).curveTo(-192.3,26,-163.8,-41.1).curveTo(-135.5,-108,-69.5,-138.8).curveTo(-3.5,-169.6,66.1,-148.4).curveTo(135.7,-127,173.2,-64.7).curveTo(210.6,-2.3,196.7,69.1).lineTo(83,47.1).curveTo(88.5,19.3,74,-5).curveTo(59.3,-29.4,32.3,-37.7).curveTo(5.2,-45.9,-20.5,-33.9).curveTo(-46.2,-21.9,-57.3,4.2).curveTo(-68.3,30.3,-59,57.1).curveTo(-49.9,83.9,-25.1,97.6).lineTo(-81.2,198.8).curveTo(-144.8,163.6,-168.5,94.8).closePath();
	var mask_graphics_350 = new cjs.Graphics().moveTo(-168,95.5).curveTo(-192.1,27,-164.1,-39.9).curveTo(-136.4,-106.7,-70.9,-137.9).curveTo(-5.5,-169,63.7,-149.1).curveTo(133,-129,171.4,-67.1).curveTo(209.8,-5.3,197.4,65.9).lineTo(83.3,45.9).curveTo(88.2,18.1,73.3,-6).curveTo(58.3,-30.1,31.4,-37.9).curveTo(4.5,-45.7,-21,-33.5).curveTo(-46.6,-21.4,-57.3,4.6).curveTo(-68.2,30.6,-58.8,57.3).curveTo(-49.6,84,-25.1,97.6).lineTo(-81.2,198.8).curveTo(-144.2,163.9,-168,95.5).closePath();
	var mask_graphics_351 = new cjs.Graphics().moveTo(-168.1,95.9).curveTo(-192,27.8,-164.7,-38.8).curveTo(-137.5,-105.5,-72.4,-137.2).curveTo(-7.3,-168.9,61.8,-149.7).curveTo(130.9,-130.5,170,-69.6).curveTo(209,-8.6,198,62.7).lineTo(83.6,44.6).curveTo(87.9,16.8,72.8,-6.9).curveTo(57.5,-30.7,30.6,-38.1).curveTo(3.7,-45.7,-21.6,-33.3).curveTo(-47,-21,-57.5,5).curveTo(-68.2,30.9,-58.9,57.5).curveTo(-49.6,84,-25.1,97.6).lineTo(-81.2,198.8).curveTo(-144.2,163.9,-168.1,95.9).closePath();
	var mask_graphics_352 = new cjs.Graphics().moveTo(-167.9,96.3).curveTo(-192,28.6,-165,-38).curveTo(-138.2,-104.6,-73.8,-136.6).curveTo(-9.6,-168.7,59.8,-150.1).curveTo(129.1,-131.6,168.8,-71.6).curveTo(208.4,-11.8,198.4,59.3).lineTo(83.8,43.3).curveTo(87.7,15.6,72.3,-7.8).curveTo(56.7,-31.1,29.8,-38.3).curveTo(2.8,-45.5,-22.2,-33.1).curveTo(-47.2,-20.6,-57.7,5.3).curveTo(-68.2,31.2,-58.8,57.6).curveTo(-49.5,84,-25.1,97.6).lineTo(-81.2,198.8).curveTo(-144,164.1,-167.9,96.3).closePath();
	var mask_graphics_353 = new cjs.Graphics().moveTo(-167.4,97).curveTo(-191.6,29.7,-165.3,-36.8).curveTo(-139.1,-103.3,-75.3,-135.7).curveTo(-11.6,-168.1,57.5,-150.7).curveTo(126.5,-133.4,167,-74.1).curveTo(207.5,-14.7,198.9,56.1).lineTo(84,42).curveTo(87.3,14.4,71.6,-8.7).curveTo(55.8,-31.8,29.1,-38.6).curveTo(2.1,-45.3,-22.6,-32.7).curveTo(-47.6,-20.1,-57.8,5.9).curveTo(-68.1,31.7,-58.6,57.9).curveTo(-49.2,84.2,-25,97.6).lineTo(-81.1,198.8).curveTo(-143.3,164.4,-167.4,97).closePath();
	var mask_graphics_354 = new cjs.Graphics().moveTo(-167.4,97.5).curveTo(-191.6,30.5,-165.9,-35.7).curveTo(-140.2,-101.9,-76.8,-134.8).curveTo(-13.4,-167.9,55.5,-151.4).curveTo(124.2,-134.8,165.4,-76.5).curveTo(206.6,-18.1,199.3,52.8).lineTo(84.2,40.7).curveTo(86.9,13.2,70.9,-9.7).curveTo(54.8,-32.3,28.1,-38.8).curveTo(1.3,-45.2,-23.3,-32.3).curveTo(-48.1,-19.5,-58,6.2).curveTo(-68,32.1,-58.6,58.1).curveTo(-49.2,84.2,-25,97.6).lineTo(-81.1,198.8).curveTo(-143.3,164.4,-167.4,97.5).closePath();
	var mask_graphics_355 = new cjs.Graphics().moveTo(-167.3,97.8).curveTo(-191.7,31.2,-166.1,-35).curveTo(-140.8,-101.1,-78.2,-134.4).curveTo(-15.7,-167.7,53.4,-151.7).curveTo(122.4,-135.8,164.2,-78.4).curveTo(205.8,-21.2,199.6,49.6).lineTo(84.3,39.4).curveTo(86.7,11.9,70.5,-10.3).curveTo(54.3,-32.7,27.4,-38.9).curveTo(0.5,-45.1,-23.9,-32.2).curveTo(-48.3,-19.2,-58.1,6.6).curveTo(-68,32.4,-58.5,58.3).curveTo(-49.2,84.2,-25,97.6).lineTo(-81.1,198.8).curveTo(-143.1,164.5,-167.3,97.8).closePath();
	var mask_graphics_356 = new cjs.Graphics().moveTo(-166.9,98.5).curveTo(-191.4,32.3,-166.6,-33.7).curveTo(-141.9,-99.7,-79.7,-133.4).curveTo(-17.7,-167,51,-152.3).curveTo(119.6,-137.5,162.2,-80.8).curveTo(204.6,-24,199.9,46.2).lineTo(84.4,38.2).curveTo(86.2,10.9,69.7,-11.3).curveTo(53.2,-33.4,26.4,-39.1).curveTo(-0.3,-44.8,-24.4,-31.7).curveTo(-48.7,-18.7,-58.3,7).curveTo(-67.9,32.8,-58.3,58.5).curveTo(-48.9,84.3,-25,97.6).lineTo(-81.1,198.8).curveTo(-142.5,164.8,-166.9,98.5).closePath();
	var mask_graphics_357 = new cjs.Graphics().moveTo(-166.8,99).curveTo(-191.3,33.1,-167.1,-32.7).curveTo(-142.9,-98.4,-81.2,-132.7).curveTo(-19.5,-166.8,48.9,-152.8).curveTo(117.4,-138.9,160.5,-83.1).curveTo(203.6,-27.4,200.1,42.9).lineTo(84.5,36.9).curveTo(85.9,9.6,69.1,-12.2).curveTo(52.4,-33.9,25.8,-39.3).curveTo(-0.9,-44.7,-25,-31.4).curveTo(-49.1,-18.1,-58.5,7.5).curveTo(-67.9,33.1,-58.4,58.6).curveTo(-48.9,84.3,-25,97.6).lineTo(-81.1,198.8).curveTo(-142.5,164.8,-166.8,99).closePath();
	var mask_graphics_358 = new cjs.Graphics().moveTo(-166.7,99.4).curveTo(-191.3,33.9,-167.4,-31.8).curveTo(-143.5,-97.5,-82.6,-132.1).curveTo(-21.7,-166.4,47,-153.1).curveTo(115.6,-139.7,159.2,-85.1).curveTo(202.7,-30.3,200.2,39.5).lineTo(84.6,35.7).curveTo(85.5,8.3,68.6,-13).curveTo(51.6,-34.2,24.9,-39.4).curveTo(-1.9,-44.6,-25.6,-31.3).curveTo(-49.3,-17.8,-58.5,7.8).curveTo(-67.9,33.4,-58.3,58.8).curveTo(-48.8,84.4,-25,97.6).lineTo(-81.1,198.8).curveTo(-142.2,165,-166.7,99.4).closePath();
	var mask_graphics_359 = new cjs.Graphics().moveTo(-166.2,100.1).curveTo(-190.9,35,-167.6,-30.6).curveTo(-144.4,-96.2,-84,-131).curveTo(-23.7,-165.7,44.5,-153.6).curveTo(112.8,-141.5,157.1,-87.3).curveTo(201.4,-33.2,200.3,36.3).lineTo(84.6,34.3).curveTo(85,7.2,67.8,-13.9).curveTo(50.5,-34.9,24,-39.7).curveTo(-2.6,-44.4,-26.1,-30.9).curveTo(-49.6,-17.3,-58.6,8.2).curveTo(-67.8,33.7,-58.1,59.1).curveTo(-48.5,84.5,-25,97.6).lineTo(-81.1,198.8).curveTo(-141.6,165.3,-166.2,100.1).closePath();
	var mask_graphics_360 = new cjs.Graphics().moveTo(-166.5,100.4).curveTo(-191.2,35.8,-168.2,-29.6).curveTo(-145.1,-94.9,-85.2,-130.2).curveTo(-25.3,-165.6,42.5,-153.9).curveTo(110.5,-142.3,155.4,-89.2).curveTo(200.4,-36.1,200.4,33.1).lineTo(84.6,33.1).curveTo(84.6,6.2,67.1,-14.6).curveTo(49.7,-35.1,23.1,-39.7).curveTo(-3.3,-44.2,-26.6,-30.5).curveTo(-49.9,-16.7,-58.9,8.7).curveTo(-67.9,34.2,-58.3,59.2).curveTo(-48.5,84.4,-24.9,97.7).lineTo(-81.1,198.9).curveTo(-141.7,165.2,-166.5,100.4).closePath();
	var mask_graphics_361 = new cjs.Graphics().moveTo(-166.2,101).curveTo(-191,36.6,-168.5,-28.6).curveTo(-146,-93.9,-86.8,-129.4).curveTo(-27.7,-164.9,40.5,-154.2).curveTo(108.6,-143.3,153.8,-91.3).curveTo(199.2,-39.2,200.4,29.9).lineTo(84.6,31.8).curveTo(84.1,4.9,66.5,-15.4).curveTo(48.9,-35.5,22.3,-39.7).curveTo(-4.2,-43.9,-27.2,-30.1).curveTo(-50.3,-16.3,-59.1,9).curveTo(-67.7,34.5,-58.1,59.5).curveTo(-48.5,84.6,-24.9,97.7).lineTo(-81.1,198.9).curveTo(-141.4,165.4,-166.2,101).closePath();
	var mask_graphics_362 = new cjs.Graphics().moveTo(-165.9,101.5).curveTo(-190.8,37.6,-168.7,-27.6).curveTo(-146.5,-92.6,-88,-128.6).curveTo(-29.5,-164.5,38.1,-154.6).curveTo(105.8,-144.6,151.9,-93.2).curveTo(197.9,-41.8,200.3,26.5).lineTo(84.7,30.5).curveTo(83.7,4,65.7,-16).curveTo(47.8,-36.1,21.4,-40).curveTo(-4.8,-43.8,-27.7,-29.9).curveTo(-50.5,-15.8,-59.1,9.5).curveTo(-67.7,34.9,-57.9,59.8).curveTo(-48.2,84.7,-24.9,97.7).lineTo(-81.1,198.9).curveTo(-140.9,165.6,-165.9,101.5).closePath();
	var mask_graphics_363 = new cjs.Graphics().moveTo(-165.8,101.9).curveTo(-190.7,38.4,-169.2,-26.5).curveTo(-147.5,-91.2,-89.4,-127.6).curveTo(-31.2,-164.1,36.1,-155).curveTo(103.4,-145.8,150,-95.4).curveTo(196.6,-44.9,200.3,23.2).lineTo(84.6,29.2).curveTo(83.3,2.8,65.1,-16.9).curveTo(47,-36.5,20.7,-40.2).curveTo(-5.4,-43.7,-28.1,-29.5).curveTo(-50.7,-15.2,-59.2,10).curveTo(-67.6,35.1,-57.9,59.9).curveTo(-48.1,84.7,-24.8,97.7).lineTo(-81,198.9).curveTo(-140.7,165.6,-165.8,101.9).closePath();
	var mask_graphics_364 = new cjs.Graphics().moveTo(-165.4,102.6).curveTo(-190.4,39.2,-169.4,-25.5).curveTo(-148.2,-90.3,-90.8,-126.8).curveTo(-33.4,-163.4,34.2,-155.1).curveTo(101.7,-146.7,148.6,-97.4).curveTo(195.5,-48,200.2,19.8).lineTo(84.7,28).curveTo(82.8,1.6,64.5,-17.7).curveTo(46.4,-36.9,19.9,-40.2).curveTo(-6.3,-43.4,-28.7,-29.2).curveTo(-51,-15,-59.3,10.2).curveTo(-67.4,35.6,-57.6,60.2).curveTo(-47.9,84.8,-24.7,97.7).lineTo(-80.9,198.9).curveTo(-140.4,166,-165.4,102.6).closePath();
	var mask_graphics_365 = new cjs.Graphics().moveTo(-165,103).curveTo(-190.1,40.2,-169.5,-24.4).curveTo(-148.8,-88.9,-91.9,-125.9).curveTo(-35,-162.7,31.9,-155.4).curveTo(98.9,-147.9,146.4,-99.3).curveTo(194.1,-50.6,200.1,16.6).lineTo(84.7,26.6).curveTo(82.4,0.6,63.8,-18.3).curveTo(45.3,-37.3,19.1,-40.3).curveTo(-6.9,-43.1,-29.1,-28.8).curveTo(-51.1,-14.4,-59.3,10.6).curveTo(-67.3,35.8,-57.5,60.3).curveTo(-47.6,84.8,-24.6,97.7).lineTo(-80.8,198.9).curveTo(-139.9,166,-165,103).closePath();
	var mask_graphics_366 = new cjs.Graphics().moveTo(-164.7,103.4).curveTo(-189.9,41,-169.7,-23.4).curveTo(-149.5,-87.6,-93.2,-124.9).curveTo(-36.7,-162.3,29.9,-155.7).curveTo(96.7,-149,144.7,-101.4).curveTo(192.8,-53.7,199.9,13.3).lineTo(84.8,25.4).curveTo(82,-0.6,63.2,-19.3).curveTo(44.5,-37.8,18.5,-40.4).curveTo(-7.3,-43,-29.3,-28.5).curveTo(-51.2,-13.9,-59.1,11.2).curveTo(-66.9,36.2,-57.2,60.5).curveTo(-47.4,84.8,-24.4,97.7).lineTo(-80.6,198.9).curveTo(-139.5,166,-164.7,103.4).closePath();
	var mask_graphics_367 = new cjs.Graphics().moveTo(-164.3,104.1).curveTo(-189.5,41.9,-169.8,-22.3).curveTo(-150.2,-86.5,-94.6,-124.1).curveTo(-38.9,-161.6,28,-155.8).curveTo(94.9,-149.8,143.1,-103.3).curveTo(191.5,-56.7,199.7,10).lineTo(84.8,24.2).curveTo(81.5,-1.8,62.7,-20).curveTo(44,-38.1,17.9,-40.4).curveTo(-8.2,-42.7,-29.8,-28.1).curveTo(-51.5,-13.5,-59.2,11.4).curveTo(-66.8,36.5,-57,60.7).curveTo(-47.2,85,-24.3,97.7).lineTo(-80.5,198.9).curveTo(-139.1,166.4,-164.3,104.1).closePath();
	var mask_graphics_368 = new cjs.Graphics().moveTo(-163.8,104.5).curveTo(-189.1,42.7,-169.8,-21.2).curveTo(-150.5,-85.1,-95.5,-123).curveTo(-40.4,-160.9,25.8,-155.9).curveTo(92.2,-150.9,141.1,-105).curveTo(190.1,-59.1,199.5,6.7).lineTo(84.9,22.8).curveTo(81.2,-2.8,62.1,-20.6).curveTo(43,-38.5,17.2,-40.6).curveTo(-8.6,-42.5,-30,-27.7).curveTo(-51.4,-12.9,-58.9,12).curveTo(-66.4,36.9,-56.6,60.9).curveTo(-46.7,85,-24,97.7).lineTo(-80.2,198.9).curveTo(-138.4,166.4,-163.8,104.5).closePath();
	var mask_graphics_369 = new cjs.Graphics().moveTo(-163.5,105).curveTo(-188.9,43.7,-170.1,-20.1).curveTo(-151.2,-83.8,-96.7,-122.1).curveTo(-41.9,-160.4,24,-156.2).curveTo(90,-152,139.3,-107.1).curveTo(188.7,-62.1,199.3,3.5).lineTo(84.8,21.6).curveTo(80.8,-4,61.5,-21.5).curveTo(42.3,-38.9,16.6,-40.6).curveTo(-9,-42.2,-30.3,-27.3).curveTo(-51.6,-12.4,-58.9,12.4).curveTo(-66.2,37.2,-56.3,61).curveTo(-46.4,85,-23.8,97.7).lineTo(-80,198.9).curveTo(-138.1,166.5,-163.5,105).closePath();
	var mask_graphics_370 = new cjs.Graphics().moveTo(-163,105.6).curveTo(-188.3,44.5,-170.1,-19.2).curveTo(-151.8,-82.8,-97.9,-121.3).curveTo(-44,-159.7,22.1,-156.2).curveTo(88.3,-152.7,137.9,-108.9).curveTo(187.5,-65.1,199,0.2).lineTo(85,20.2).curveTo(80.5,-5.1,61.1,-22.1).curveTo(41.9,-39.2,16,-40.6).curveTo(-9.7,-41.9,-30.7,-27).curveTo(-51.6,-12,-58.7,12.8).curveTo(-65.8,37.6,-56,61.3).curveTo(-46.1,85.1,-23.5,97.7).lineTo(-79.7,198.9).curveTo(-137.5,166.8,-163,105.6).closePath();
	var mask_graphics_371 = new cjs.Graphics().moveTo(-162.4,106.1).curveTo(-187.9,45.4,-170,-18.1).curveTo(-152,-81.5,-98.7,-120.2).curveTo(-45.4,-158.9,20,-156.3).curveTo(85.6,-153.8,135.8,-110.6).curveTo(186,-67.4,198.6,-3).lineTo(85,19).curveTo(80.1,-6,60.5,-22.8).curveTo(41,-39.6,15.4,-40.7).curveTo(-10,-41.6,-30.8,-26.6).curveTo(-51.5,-11.4,-58.5,13.2).curveTo(-65.4,37.9,-55.5,61.6).curveTo(-45.5,85.3,-23.2,97.7).lineTo(-79.4,198.9).curveTo(-136.8,166.9,-162.4,106.1).closePath();
	var mask_graphics_372 = new cjs.Graphics().moveTo(-161.9,106.5).curveTo(-187.4,46.2,-170,-16.9).curveTo(-152.6,-80,-99.8,-119.2).curveTo(-46.8,-158.4,18.3,-156.5).curveTo(83.5,-154.6,134,-112.5).curveTo(184.6,-70.4,198.3,-6.3).lineTo(85.1,17.8).curveTo(79.8,-7.1,60,-23.6).curveTo(40.4,-40,14.9,-40.7).curveTo(-10.4,-41.4,-31,-26.2).curveTo(-51.6,-10.9,-58.3,13.6).curveTo(-65.1,38.3,-55.2,61.7).curveTo(-45.2,85.3,-22.8,97.7).lineTo(-79,198.9).curveTo(-136.3,166.9,-161.9,106.5).closePath();
	var mask_graphics_373 = new cjs.Graphics().moveTo(-161.3,107.2).curveTo(-186.8,47.1,-170,-16).curveTo(-153.1,-79,-100.9,-118.3).curveTo(-48.8,-157.6,16.5,-156.5).curveTo(81.8,-155.3,132.6,-114.2).curveTo(183.3,-73.2,198,-9.5).lineTo(85.2,16.5).curveTo(79.5,-8.2,59.7,-24.3).curveTo(39.9,-40.3,14.5,-40.7).curveTo(-10.9,-41.1,-31.2,-25.8).curveTo(-51.5,-10.5,-58.1,14).curveTo(-64.6,38.5,-54.7,62).curveTo(-44.7,85.4,-22.5,97.7).lineTo(-78.7,198.9).curveTo(-135.7,167.3,-161.3,107.2).closePath();
	var mask_graphics_374 = new cjs.Graphics().moveTo(-160.6,107.6).curveTo(-186.2,48,-169.7,-14.8).curveTo(-153.2,-77.7,-101.6,-117.2).curveTo(-50,-156.8,14.6,-156.5).curveTo(79.2,-156.2,130.5,-115.9).curveTo(181.8,-75.5,197.6,-12.8).lineTo(85.3,15.2).curveTo(79.1,-9.1,59,-24.8).curveTo(39.1,-40.6,13.9,-40.7).curveTo(-11.1,-40.8,-31.2,-25.4).curveTo(-51.2,-10,-57.7,14.4).curveTo(-64.2,38.9,-54.2,62.1).curveTo(-44.2,85.4,-22.1,97.7).lineTo(-78.3,198.9).curveTo(-134.9,167.3,-160.6,107.6).closePath();
	var mask_graphics_375 = new cjs.Graphics().moveTo(-160.1,108).curveTo(-185.7,48.8,-169.7,-13.7).curveTo(-153.6,-76.2,-102.5,-116.3).curveTo(-51.4,-156.2,12.8,-156.6).curveTo(77,-157,128.6,-117.7).curveTo(180.4,-78.3,197.2,-15.9).lineTo(85.3,14).curveTo(78.8,-10.2,58.6,-25.5).curveTo(38.6,-40.8,13.5,-40.7).curveTo(-11.4,-40.6,-31.3,-25).curveTo(-51.2,-9.4,-57.4,14.8).curveTo(-63.7,39.2,-53.8,62.2).curveTo(-43.8,85.4,-21.7,97.7).lineTo(-77.9,198.9).curveTo(-134.4,167.3,-160.1,108).closePath();
	var mask_graphics_376 = new cjs.Graphics().moveTo(-167.8,85.3).curveTo(-189.5,9.8,-151.4,-58.8).curveTo(-113.2,-127.5,-37.8,-149.2).curveTo(37.8,-170.8,106.4,-132.8).curveTo(175.1,-94.6,196.7,-19.2).lineTo(85.4,12.8).curveTo(77,-16.6,50.2,-31.5).curveTo(23.5,-46.2,-5.8,-37.8).curveTo(-35.2,-29.5,-50.1,-2.8).curveTo(-64.9,24,-56.5,53.4).curveTo(-48.1,82.8,-21.3,97.7).lineTo(-77.5,198.9).curveTo(-146.1,160.8,-167.8,85.3).closePath();
	var mask_graphics_377 = new cjs.Graphics().moveTo(-167.1,86.2).curveTo(-189,11.4,-151.6,-57.2).curveTo(-114.1,-125.9,-39.4,-148.5).curveTo(35.5,-171,104.3,-133.9).curveTo(173.3,-96.8,196.3,-22.3).lineTo(85.5,11.6).curveTo(76.6,-17.4,49.7,-31.9).curveTo(22.9,-46.4,-6.2,-37.6).curveTo(-35.3,-28.8,-49.9,-2.1).curveTo(-64.4,24.7,-55.9,53.7).curveTo(-47.4,82.8,-20.8,97.7).lineTo(-77,198.9).curveTo(-145.1,161,-167.1,86.2).closePath();
	var mask_graphics_378 = new cjs.Graphics().moveTo(-166.7,86.6).curveTo(-188.6,12.4,-151.9,-56.3).curveTo(-115.1,-124.8,-40.9,-148.1).curveTo(33.5,-171.2,102.6,-135.4).curveTo(171.6,-99.5,195.7,-25.5).lineTo(85.6,10.4).curveTo(76.3,-18.5,49.3,-32.4).curveTo(22.5,-46.4,-6.5,-37.4).curveTo(-35.3,-28.4,-49.7,-1.7).curveTo(-63.9,25.1,-55.3,54).curveTo(-46.8,82.8,-20.3,97.7).lineTo(-76.5,198.9).curveTo(-144.6,161,-166.7,86.6).closePath();
	var mask_graphics_379 = new cjs.Graphics().moveTo(-165.6,87.3).curveTo(-187.9,13.3,-151.8,-55).curveTo(-115.7,-123.3,-42.3,-147.3).curveTo(31.3,-171.1,100.6,-136.3).curveTo(169.9,-101.4,195.3,-28.6).lineTo(85.7,9.1).curveTo(75.9,-19.3,48.9,-32.8).curveTo(21.9,-46.4,-6.7,-37.2).curveTo(-35.2,-27.8,-49.3,-1.3).curveTo(-63.4,25.4,-54.7,54.2).curveTo(-46.1,83.1,-19.8,97.7).lineTo(-76,198.9).curveTo(-143.3,161.4,-165.6,87.3).closePath();
	var mask_graphics_380 = new cjs.Graphics().moveTo(-165.1,87.8).curveTo(-187.3,14.3,-152,-54).curveTo(-116.5,-122.2,-43.6,-146.7).curveTo(29.2,-171.2,98.7,-137.7).curveTo(168.2,-103.9,194.7,-31.8).lineTo(85.8,7.9).curveTo(75.5,-20.2,48.5,-33.4).curveTo(21.4,-46.4,-6.9,-36.9).curveTo(-35.2,-27.3,-49,-0.7).curveTo(-62.9,25.8,-54.2,54.4).curveTo(-45.5,83.1,-19.3,97.7).lineTo(-75.5,198.9).curveTo(-142.8,161.4,-165.1,87.8).closePath();
	var mask_graphics_381 = new cjs.Graphics().moveTo(-164.2,88.5).curveTo(-186.6,15.2,-151.9,-53).curveTo(-117.1,-121.3,-44.6,-146.2).curveTo(27.8,-171.1,97.1,-138.7).curveTo(166.6,-106.4,194.1,-34.9).lineTo(86,6.7).curveTo(75.3,-21.2,48.3,-33.8).curveTo(21.3,-46.4,-7,-36.8).curveTo(-35.2,-27,-48.7,-0.5).curveTo(-62.3,26.2,-53.6,54.6).curveTo(-44.8,83.2,-18.6,97.7).lineTo(-74.8,198.9).curveTo(-141.7,161.8,-164.2,88.5).closePath();
	var mask_graphics_382 = new cjs.Graphics().moveTo(-162.7,89.9).curveTo(-185.5,17.8,-152,-50.3).curveTo(-118.5,-118.3,-47.4,-144.8).curveTo(23.7,-171.2,93.3,-141).curveTo(163,-110.8,192.9,-41).lineTo(86.3,4.3).curveTo(74.7,-23,47.6,-34.7).curveTo(20.5,-46.4,-7.3,-36.1).curveTo(-34.9,-25.8,-48,0.6).curveTo(-61,27.2,-52.2,55.2).curveTo(-43.3,83.2,-17.4,97.7).lineTo(-73.6,198.9).curveTo(-140,161.9,-162.7,89.9).closePath();
	var mask_graphics_383 = new cjs.Graphics().moveTo(-161.7,90.5).curveTo(-184.5,18.8,-151.8,-49.1).curveTo(-118.9,-116.9,-48.6,-144).curveTo(21.8,-171,91.6,-141.9).curveTo(161.5,-112.6,192.2,-43.9).lineTo(86.4,3).curveTo(74.5,-23.6,47.3,-35).curveTo(20.1,-46.4,-7.3,-35.8).curveTo(-34.6,-25.3,-47.5,1.2).curveTo(-60.2,27.6,-51.3,55.5).curveTo(-42.3,83.4,-16.7,97.7).lineTo(-73,198.9).curveTo(-138.6,162.3,-161.7,90.5).closePath();
	var mask_graphics_384 = new cjs.Graphics().moveTo(-160.8,90.9).curveTo(-183.9,19.7,-151.8,-48).curveTo(-119.5,-115.7,-49.9,-143.3).curveTo(19.8,-171,89.7,-143.1).curveTo(159.6,-115,191.5,-47.1).lineTo(86.6,2).curveTo(74.1,-24.6,46.9,-35.5).curveTo(19.8,-46.4,-7.4,-35.7).curveTo(-34.5,-24.8,-47.1,1.6).curveTo(-59.5,28,-50.6,55.6).curveTo(-41.7,83.4,-16.1,97.7).lineTo(-72.3,198.9).curveTo(-137.8,162.3,-160.8,90.9).closePath();
	var mask_graphics_385 = new cjs.Graphics().moveTo(-160,91.6).curveTo(-183,20.7,-151.4,-47.1).curveTo(-119.9,-114.6,-50.7,-142.7).curveTo(18.7,-170.7,88.4,-143.9).curveTo(158.2,-117.1,190.8,-50).lineTo(86.8,0.7).curveTo(74.1,-25.4,46.8,-35.8).curveTo(19.7,-46.2,-7.2,-35.4).curveTo(-34.2,-24.4,-46.5,1.8).curveTo(-58.8,28.2,-49.9,55.9).curveTo(-40.8,83.6,-15.3,97.7).lineTo(-71.5,198.9).curveTo(-136.8,162.7,-160,91.6).closePath();
	var mask_graphics_386 = new cjs.Graphics().moveTo(-158.9,92.4).curveTo(-182.2,22.3,-151.4,-45.4).curveTo(-120.3,-113,-51.6,-141.9).curveTo(17.4,-170.6,86.7,-144.8).curveTo(156.2,-119,190,-53).lineTo(87,-0.3).curveTo(73.7,-26.1,46.6,-36.2).curveTo(19.7,-46.2,-7.1,-35).curveTo(-33.9,-23.8,-46,2.5).curveTo(-58.1,28.9,-49,56.3).curveTo(-39.9,83.6,-14.6,97.7).lineTo(-70.8,198.9).curveTo(-135.5,162.7,-158.9,92.4).closePath();
	var mask_graphics_387 = new cjs.Graphics().moveTo(-158.2,93).curveTo(-181.5,23.2,-151.2,-44.2).curveTo(-120.8,-111.7,-52.7,-141).curveTo(15.5,-170.4,84.9,-145.9).curveTo(154.4,-121.3,189.3,-55.9).lineTo(87.1,-1.6).curveTo(73.5,-27,46.4,-36.6).curveTo(19.5,-46.1,-7.1,-34.7).curveTo(-33.6,-23.2,-45.5,3).curveTo(-57.3,29.3,-48.2,56.4).curveTo(-39,83.6,-13.8,97.7).lineTo(-70,198.9).curveTo(-134.8,162.8,-158.2,93).closePath();
	var mask_graphics_388 = new cjs.Graphics().moveTo(-156.8,93.6).curveTo(-180.4,24.2,-150.8,-43.1).curveTo(-121.1,-110.3,-53.7,-140.2).curveTo(13.8,-170,83.3,-146.5).curveTo(152.9,-122.8,188.5,-58.8).lineTo(87.3,-2.6).curveTo(73.4,-27.6,46.3,-36.8).curveTo(19.2,-46,-7.1,-34.3).curveTo(-33.2,-22.7,-44.9,3.5).curveTo(-56.4,29.6,-47.2,56.7).curveTo(-37.9,83.8,-13,97.7).lineTo(-69.2,198.9).curveTo(-133.1,163.3,-156.8,93.6).closePath();
	var mask_graphics_389 = new cjs.Graphics().moveTo(-156,94.2).curveTo(-179.6,25.1,-150.6,-41.9).curveTo(-121.5,-108.9,-54.7,-139.4).curveTo(12,-169.9,81.5,-147.4).curveTo(151,-124.9,187.7,-61.7).lineTo(87.5,-3.7).curveTo(73.1,-28.4,46,-37.2).curveTo(19.1,-46,-6.9,-34.1).curveTo(-32.9,-22.1,-44.3,3.9).curveTo(-55.5,30,-46.3,56.8).curveTo(-37.1,83.8,-12.2,97.7).lineTo(-68.4,198.9).curveTo(-132.3,163.3,-156,94.2).closePath();
	var mask_graphics_390 = new cjs.Graphics().moveTo(-155,94.9).curveTo(-178.6,26.1,-150.2,-41).curveTo(-121.8,-108,-55.8,-138.7).curveTo(10.1,-169.5,79.7,-148.2).curveTo(149.3,-127,186.9,-64.5).lineTo(87.6,-4.9).curveTo(73,-29.2,45.9,-37.4).curveTo(18.8,-45.7,-6.9,-33.8).curveTo(-32.5,-21.9,-43.6,4.3).curveTo(-54.6,30.4,-45.4,57.1).curveTo(-36.2,83.9,-11.4,97.7).lineTo(-67.6,198.9).curveTo(-131.3,163.7,-155,94.9).closePath();
	var mask_graphics_391 = new cjs.Graphics().moveTo(-153.8,95.5).curveTo(-177.8,27.6,-150,-39.3).curveTo(-122.1,-106.2,-56.6,-137.8).curveTo(9.1,-169.2,78.3,-148.9).curveTo(147.5,-128.6,186,-67.4).lineTo(87.8,-6).curveTo(72.8,-29.9,45.8,-37.8).curveTo(19,-45.7,-6.6,-33.4).curveTo(-32.1,-21.1,-42.9,4.9).curveTo(-53.7,30.9,-44.5,57.4).curveTo(-35.2,83.9,-10.5,97.7).lineTo(-66.7,198.9).curveTo(-129.8,163.7,-153.8,95.5).closePath();
	var mask_graphics_392 = new cjs.Graphics().moveTo(-152.9,96.1).curveTo(-176.9,28.5,-149.7,-38.3).curveTo(-122.3,-104.9,-57.4,-137).curveTo(7.4,-168.9,76.5,-149.8).curveTo(145.6,-130.6,185.1,-70.1).lineTo(88,-7.1).curveTo(72.6,-30.7,45.6,-38.1).curveTo(18.8,-45.6,-6.5,-33.1).curveTo(-31.7,-20.6,-42.3,5.4).curveTo(-52.8,31.4,-43.5,57.6).curveTo(-34.2,84,-9.6,97.7).lineTo(-65.8,198.9).curveTo(-128.8,163.8,-152.9,96.1).closePath();
	var mask_graphics_393 = new cjs.Graphics().moveTo(-151.5,96.8).curveTo(-175.7,29.5,-149.2,-37).curveTo(-122.5,-103.4,-58.5,-135.9).curveTo(5.7,-168.4,74.8,-150.3).curveTo(144,-132,184.2,-72.9).lineTo(88.2,-8.2).curveTo(72.5,-31.2,45.5,-38.3).curveTo(18.7,-45.3,-6.3,-32.7).curveTo(-31.2,-20,-41.7,5.9).curveTo(-52,31.8,-42.5,57.9).curveTo(-33,84.2,-8.8,97.7).lineTo(-65,198.9).curveTo(-127.3,164.1,-151.5,96.8).closePath();
	var mask_graphics_394 = new cjs.Graphics().moveTo(-150.5,97.3).curveTo(-174.8,30.4,-148.8,-35.8).curveTo(-122.7,-102,-59.3,-135.1).curveTo(4.1,-168.1,73,-151.1).curveTo(142.1,-134,183.3,-75.6).lineTo(88.3,-9.3).curveTo(72.4,-31.9,45.4,-38.5).curveTo(18.6,-45.2,-6.1,-32.3).curveTo(-30.7,-19.4,-40.9,6.3).curveTo(-51,32,-41.5,58).curveTo(-32.1,84.2,-7.8,97.7).lineTo(-64,198.9).curveTo(-126.3,164.2,-150.5,97.3).closePath();
	var mask_graphics_395 = new cjs.Graphics().moveTo(-149.3,98).curveTo(-173.6,31.4,-148.1,-34.9).curveTo(-122.7,-101,-60.1,-134.3).curveTo(2.5,-167.6,71.5,-151.6).curveTo(140.6,-135.6,182.3,-78.3).lineTo(88.6,-10.2).curveTo(72.4,-32.6,45.4,-38.8).curveTo(18.6,-45,-5.8,-32.2).curveTo(-30.2,-19.2,-40,6.6).curveTo(-49.9,32.4,-40.4,58.3).curveTo(-31,84.3,-6.9,97.7).lineTo(-63.1,198.9).curveTo(-125.1,164.6,-149.3,98).closePath();
	var mask_graphics_396 = new cjs.Graphics().moveTo(-148,98.8).curveTo(-172.4,33,-147.6,-33.1).curveTo(-122.9,-99.2,-60.7,-133.2).curveTo(1.6,-167.2,70.1,-152.1).curveTo(138.8,-137.1,181.3,-81).lineTo(88.8,-11.3).curveTo(72.3,-33.1,45.6,-39.1).curveTo(18.9,-44.9,-5.3,-31.6).curveTo(-29.5,-18.3,-39.3,7.2).curveTo(-48.9,33,-39.4,58.6).curveTo(-29.8,84.3,-5.8,97.7).lineTo(-62,198.9).curveTo(-123.5,164.6,-148,98.8).closePath();
	var mask_graphics_397 = new cjs.Graphics().moveTo(-146.9,98.9).curveTo(-171.4,33.2,-147.2,-32.3).curveTo(-122.9,-97.8,-61.4,-132.4).curveTo(0,-166.8,68.4,-153).curveTo(136.8,-139,180.3,-83.6).lineTo(89,-12.4).curveTo(72.1,-33.9,45.4,-39.3).curveTo(18.9,-44.8,-5.1,-31.3).curveTo(-28.9,-17.8,-38.4,7.7).curveTo(-47.9,33.1,-38.4,58.7).curveTo(-28.8,84.3,-4.8,97.7).lineTo(-61,198.9).curveTo(-122.4,164.6,-146.9,98.9).closePath();
	var mask_graphics_398 = new cjs.Graphics().moveTo(-145.5,99.5).curveTo(-170.1,34.1,-146.6,-31.1).curveTo(-122.9,-96.2,-62.2,-131.2).curveTo(-1.5,-166.1,66.9,-153.2).curveTo(135.3,-140.2,179.3,-86.2).lineTo(89.4,-13.3).curveTo(72.2,-34.3,45.5,-39.5).curveTo(18.9,-44.5,-4.8,-30.9).curveTo(-28.3,-17.3,-37.5,8.1).curveTo(-46.7,33.5,-37.1,59).curveTo(-27.5,84.4,-3.8,97.7).lineTo(-60,198.9).curveTo(-120.7,165,-145.5,99.5).closePath();
	var mask_graphics_399 = new cjs.Graphics().moveTo(-144.3,100).curveTo(-169,35,-146,-30).curveTo(-122.9,-94.9,-63,-130.2).curveTo(-3,-165.6,65.1,-153.8).curveTo(133.4,-141.9,178.2,-88.8).lineTo(89.5,-14.3).curveTo(72.2,-35,45.6,-39.6).curveTo(19.1,-44.2,-4.4,-30.5).curveTo(-27.7,-16.7,-36.7,8.6).curveTo(-45.7,33.9,-36,59.1).curveTo(-26.3,84.4,-2.7,97.7).lineTo(-58.9,198.9).curveTo(-119.5,165.2,-144.3,100).closePath();
	var mask_graphics_400 = new cjs.Graphics().moveTo(-143,101).curveTo(-167.7,36.6,-145.3,-28.6).curveTo(-122.8,-93.9,-63.6,-129.4).curveTo(-4.4,-164.9,63.7,-154.2).curveTo(131.8,-143.3,177.2,-91.2).lineTo(89.7,-15.4).curveTo(72.1,-35.5,45.5,-39.7).curveTo(19,-43.9,-4,-30.1).curveTo(-27,-16.3,-35.8,9).curveTo(-44.5,34.5,-34.9,59.5).curveTo(-25.3,84.6,-1.7,97.7).lineTo(-57.9,198.9).curveTo(-118.2,165.4,-143,101).closePath();
	var mask_graphics_401 = new cjs.Graphics().moveTo(-141.5,101.5).curveTo(-166.5,37.6,-144.7,-27.2).curveTo(-122.7,-91.9,-63.9,-128.2).curveTo(-5.2,-164.3,62.4,-154.6).curveTo(130,-144.7,176,-93.8).lineTo(90,-16.3).curveTo(72,-36.1,45.7,-40).curveTo(19.5,-43.8,-3.4,-29.7).curveTo(-26.3,-15.6,-34.8,9.5).curveTo(-43.4,34.9,-33.6,59.8).curveTo(-23.9,84.7,-0.6,97.7).lineTo(-56.8,198.9).curveTo(-116.6,165.6,-141.5,101.5).closePath();
	var mask_graphics_402 = new cjs.Graphics().moveTo(-140.5,102).curveTo(-165.4,38.5,-144.1,-26.1).curveTo(-122.7,-90.5,-64.8,-127.2).curveTo(-6.7,-163.8,60.6,-155).curveTo(128,-146.2,174.9,-96.2).lineTo(90.3,-17.3).curveTo(72,-36.8,45.7,-40.2).curveTo(19.6,-43.5,-3,-29.3).curveTo(-25.6,-15,-34,10.1).curveTo(-42.3,35.1,-32.5,59.9).curveTo(-22.8,84.7,0.5,97.7).lineTo(-55.7,198.9).curveTo(-115.4,165.6,-140.5,102).closePath();
	var mask_graphics_403 = new cjs.Graphics().moveTo(-138.8,102.6).curveTo(-163.8,39.3,-143.1,-24.8).curveTo(-122.4,-89,-65.2,-126).curveTo(-7.9,-163,59.2,-155.1).curveTo(126.5,-147.3,173.8,-98.7).lineTo(90.5,-18.2).curveTo(72.1,-37,46,-40.2).curveTo(19.8,-43.3,-2.5,-28.9).curveTo(-24.7,-14.4,-32.9,10.5).curveTo(-41,35.6,-31.2,60.2).curveTo(-21.4,84.8,1.7,97.7).lineTo(-54.5,198.9).curveTo(-113.6,166,-138.8,102.6).closePath();
	var mask_graphics_404 = new cjs.Graphics().moveTo(-137.5,103.1).curveTo(-162.7,40.3,-142.5,-23.6).curveTo(-122.3,-87.5,-65.8,-124.9).curveTo(-9.4,-162.3,57.5,-155.5).curveTo(124.6,-148.8,172.6,-101).lineTo(90.7,-19).curveTo(72,-37.7,45.9,-40.4).curveTo(19.9,-43,-2.1,-28.5).curveTo(-24,-13.9,-31.8,11).curveTo(-39.7,36,-30,60.3).curveTo(-20.2,84.8,2.8,97.7).lineTo(-53.4,198.9).curveTo(-112.3,166,-137.5,103.1).closePath();
	var mask_graphics_405 = new cjs.Graphics().moveTo(-136.1,104.1).curveTo(-161.2,41.9,-141.6,-22.3).curveTo(-122,-86.5,-66.3,-124.1).curveTo(-10.7,-161.6,56.2,-155.8).curveTo(123.1,-149.8,171.5,-103.3).lineTo(91,-20).curveTo(72.2,-38.1,46.1,-40.4).curveTo(20.1,-42.7,-1.6,-28.1).curveTo(-23.3,-13.5,-31,11.4).curveTo(-38.6,36.5,-28.8,60.7).curveTo(-18.9,85,4,97.7).lineTo(-52.2,198.9).curveTo(-110.9,166.4,-136.1,104.1).closePath();
	var mask_graphics_406 = new cjs.Graphics().moveTo(-134.6,104.6).curveTo(-159.9,42.9,-140.8,-20.9).curveTo(-121.7,-84.6,-66.6,-122.8).curveTo(-11.3,-160.9,54.9,-156.1).curveTo(121.2,-151.1,170.3,-105.6).lineTo(91.3,-20.9).curveTo(72.2,-38.5,46.4,-40.6).curveTo(20.6,-42.5,-0.9,-27.6).curveTo(-22.4,-12.7,-29.9,12.1).curveTo(-37.2,36.9,-27.4,60.9).curveTo(-17.6,85,5.2,97.7).lineTo(-51,198.9).curveTo(-109.2,166.4,-134.6,104.6).closePath();
	var mask_graphics_407 = new cjs.Graphics().moveTo(-133.2,105).curveTo(-158.5,43.7,-140.1,-19.7).curveTo(-121.6,-83.1,-67.1,-121.7).curveTo(-12.6,-160.1,53.3,-156.2).curveTo(119.2,-152.3,169.1,-107.7).lineTo(91.6,-21.7).curveTo(72.2,-39.1,46.5,-40.7).curveTo(20.9,-42.2,-0.4,-27.2).curveTo(-21.6,-12.1,-28.8,12.5).curveTo(-36,37.2,-26.1,61).curveTo(-16.2,85,6.4,97.7).lineTo(-49.8,198.9).curveTo(-107.9,166.5,-133.2,105).closePath();
	var mask_graphics_408 = new cjs.Graphics().moveTo(-131.5,105.7).curveTo(-157,44.6,-139.1,-18.5).curveTo(-121.1,-81.6,-67.5,-120.5).curveTo(-13.7,-159.2,52.1,-156.2).curveTo(117.9,-153.1,167.8,-110).lineTo(91.9,-22.5).curveTo(72.4,-39.3,46.8,-40.6).curveTo(21.2,-41.8,0.2,-26.7).curveTo(-20.6,-11.6,-27.7,12.9).curveTo(-34.6,37.6,-24.7,61.3).curveTo(-14.7,85.1,7.7,97.7).lineTo(-48.5,198.9).curveTo(-105.9,166.9,-131.5,105.7).closePath();
	var mask_graphics_409 = new cjs.Graphics().moveTo(-130.2,106.2).curveTo(-155.6,45.6,-138.3,-17.3).curveTo(-120.8,-80.1,-67.9,-119.2).curveTo(-14.9,-158.4,50.5,-156.3).curveTo(115.9,-154.3,166.5,-112.1).lineTo(92.1,-23.4).curveTo(72.4,-39.9,47,-40.7).curveTo(21.5,-41.5,0.8,-26.2).curveTo(-19.8,-10.9,-26.6,13.5).curveTo(-33.3,38,-23.5,61.6).curveTo(-13.4,85.3,8.9,97.7).lineTo(-47.3,198.9).curveTo(-104.6,166.9,-130.2,106.2).closePath();
	var mask_graphics_410 = new cjs.Graphics().moveTo(-128.6,107.2).curveTo(-154.1,47.1,-137.3,-16).curveTo(-120.3,-79,-68.2,-118.3).curveTo(-16.1,-157.6,49.2,-156.5).curveTo(114.5,-155.3,165.3,-114.2).lineTo(92.4,-24.3).curveTo(72.6,-40.3,47.2,-40.7).curveTo(21.8,-41.1,1.5,-25.8).curveTo(-18.8,-10.5,-25.4,14).curveTo(-31.9,38.5,-22,62).curveTo(-12,85.4,10.2,97.7).lineTo(-46,198.9).curveTo(-103,167.3,-128.6,107.2).closePath();
	var mask_graphics_411 = new cjs.Graphics().moveTo(-127.1,107.6).curveTo(-152.6,48,-136.4,-14.6).curveTo(-120,-77,-68.3,-116.9).curveTo(-16.5,-156.8,48.1,-156.5).curveTo(112.6,-156.2,164,-116.3).lineTo(92.7,-25).curveTo(72.7,-40.6,47.5,-40.7).curveTo(22.5,-40.8,2.3,-25.4).curveTo(-17.9,-9.8,-24.3,14.6).curveTo(-30.6,38.9,-20.6,62.1).curveTo(-10.6,85.4,11.5,97.7).lineTo(-44.7,198.9).curveTo(-101.3,167.3,-127.1,107.6).closePath();
	var mask_graphics_412 = new cjs.Graphics().moveTo(-125.6,108.1).curveTo(-151.2,49,-135.4,-13.3).curveTo(-119.5,-75.5,-68.6,-115.7).curveTo(-17.7,-155.9,46.5,-156.6).curveTo(110.7,-157.3,162.7,-118.3).lineTo(92.9,-25.8).curveTo(72.8,-41,47.7,-40.7).curveTo(22.8,-40.4,2.9,-24.8).curveTo(-16.9,-9.1,-23.1,15).curveTo(-29.2,39.2,-19.3,62.2).curveTo(-9.3,85.4,12.8,97.7).lineTo(-43.4,198.9).curveTo(-99.9,167.3,-125.6,108.1).closePath();
	var mask_graphics_413 = new cjs.Graphics().moveTo(-123.7,108.7).curveTo(-149.6,49.8,-134.3,-12.1).curveTo(-119,-73.9,-68.9,-114.4).curveTo(-18.6,-154.7,45.3,-156.3).curveTo(109.3,-157.8,161.3,-120.3).lineTo(93.2,-26.6).curveTo(73,-41.2,48.1,-40.7).curveTo(23.2,-40,3.7,-24.3).curveTo(-15.8,-8.6,-21.8,15.5).curveTo(-27.7,39.6,-17.7,62.5).curveTo(-7.7,85.5,14.1,97.7).lineTo(-42.1,198.9).curveTo(-97.9,167.7,-123.7,108.7).closePath();
	var mask_graphics_414 = new cjs.Graphics().moveTo(-122.2,109.2).curveTo(-148.1,50.7,-133.4,-10.9).curveTo(-118.6,-72.4,-69.2,-113.1).curveTo(-19.7,-153.8,43.8,-156.3).curveTo(107.4,-158.8,160,-122.2).lineTo(93.6,-27.3).curveTo(73,-41.6,48.3,-40.7).curveTo(23.6,-39.6,4.4,-23.8).curveTo(-14.9,-7.9,-20.7,16).curveTo(-26.4,40,-16.3,62.8).curveTo(-6.2,85.5,15.5,97.7).lineTo(-40.7,198.9).curveTo(-96.4,167.9,-122.2,109.2).closePath();
	var mask_graphics_415 = new cjs.Graphics().moveTo(-120.6,110.2).curveTo(-146.5,52.2,-132.3,-9.5).curveTo(-117.9,-71.3,-69.3,-112.2).curveTo(-20.7,-153,42.7,-156.3).curveTo(106.1,-159.6,158.6,-124.1).lineTo(93.9,-28.1).curveTo(73.4,-41.9,48.7,-40.7).curveTo(24,-39.3,5.1,-23.5).curveTo(-13.8,-7.5,-19.3,16.5).curveTo(-24.9,40.6,-14.9,63).curveTo(-4.8,85.7,16.8,97.7).lineTo(-39.4,198.9).curveTo(-94.8,168.1,-120.6,110.2).closePath();
	var mask_graphics_416 = new cjs.Graphics().moveTo(-118.9,110.7).curveTo(-144.8,53.2,-131.1,-8.1).curveTo(-117.3,-69.3,-69.2,-110.6).curveTo(-21,-151.9,41.6,-156.1).curveTo(104.1,-160.3,157.2,-125.9).lineTo(94.2,-28.8).curveTo(73.5,-42.2,49.1,-40.6).curveTo(24.8,-38.9,5.9,-22.8).curveTo(-12.8,-6.7,-18.2,17.1).curveTo(-23.5,41,-13.3,63.3).curveTo(-3.1,85.7,18.3,97.7).lineTo(-37.9,198.9).curveTo(-92.9,168.3,-118.9,110.7).closePath();
	var mask_graphics_417 = new cjs.Graphics().moveTo(-117.4,111.1).curveTo(-143.3,54.1,-130.2,-6.8).curveTo(-116.9,-67.8,-69.5,-109.4).curveTo(-22.1,-150.9,40.1,-156.1).curveTo(102.2,-161.1,155.8,-127.6).lineTo(94.5,-29.5).curveTo(73.6,-42.5,49.4,-40.6).curveTo(25.2,-38.5,6.7,-22.4).curveTo(-11.7,-6.2,-16.8,17.5).curveTo(-22,41.2,-11.9,63.4).curveTo(-1.8,85.8,19.6,97.7).lineTo(-36.6,198.9).curveTo(-91.4,168.3,-117.4,111.1).closePath();
	var mask_graphics_418 = new cjs.Graphics().moveTo(-115.5,111.8).curveTo(-141.6,54.9,-128.9,-5.6).curveTo(-116.2,-66.2,-69.6,-108).curveTo(-22.8,-149.7,39,-155.7).curveTo(100.9,-161.5,154.4,-129.4).lineTo(94.8,-30.1).curveTo(74,-42.6,49.9,-40.3).curveTo(25.8,-38,7.6,-21.7).curveTo(-10.5,-5.5,-15.5,18.1).curveTo(-20.4,41.6,-10.2,63.7).curveTo(-0.1,85.9,21,97.7).lineTo(-35.2,198.9).curveTo(-89.3,168.7,-115.5,111.8).closePath();
	var mask_graphics_419 = new cjs.Graphics().moveTo(-113.9,112.2).curveTo(-140.1,55.9,-127.9,-4.4).curveTo(-115.5,-64.7,-69.8,-106.6).curveTo(-23.9,-148.6,37.5,-155.4).curveTo(99,-162.2,153,-131).lineTo(95.2,-30.8).curveTo(74.1,-42.9,50.1,-40.3).curveTo(26.2,-37.6,8.4,-21.3).curveTo(-9.4,-4.9,-14.2,18.5).curveTo(-19,41.9,-8.8,63.9).curveTo(1.3,85.9,22.5,97.7).lineTo(-33.7,198.9).curveTo(-87.8,168.7,-113.9,112.2).closePath();
	var mask_graphics_420 = new cjs.Graphics().moveTo(-122.7,85.3).curveTo(-144.3,9.8,-106.3,-58.8).curveTo(-68.1,-127.5,7.4,-149.2).curveTo(82.9,-170.8,151.6,-132.6).lineTo(95.4,-31.5).curveTo(68.7,-46.2,39.3,-37.8).curveTo(9.9,-29.5,-5,-2.8).curveTo(-19.7,24,-11.3,53.4).curveTo(-2.9,82.8,23.9,97.7).lineTo(-32.3,198.9).curveTo(-101,160.8,-122.7,85.3).closePath();
	var mask_graphics_421 = new cjs.Graphics().moveTo(-120.9,86.2).curveTo(-142.8,11.6,-105.5,-57.2).curveTo(-68.1,-125.9,6.4,-148.5).curveTo(80.9,-171,150.1,-134.3).lineTo(95.8,-32).curveTo(68.8,-46.4,39.9,-37.6).curveTo(10.9,-28.8,-3.7,-2.1).curveTo(-18.2,24.7,-9.7,53.7).curveTo(-1.2,82.8,25.4,97.7).lineTo(-30.8,198.9).curveTo(-99,161,-120.9,86.2).closePath();
	var mask_graphics_422 = new cjs.Graphics().moveTo(-119.3,86.9).curveTo(-141.3,12.5,-104.8,-56).curveTo(-68.4,-124.4,5.5,-147.7).curveTo(79.5,-171,148.7,-135.8).lineTo(96,-32.7).curveTo(69.2,-46.4,40.3,-37.3).curveTo(11.5,-28.2,-2.7,-1.6).curveTo(-16.8,25.1,-8.3,54.1).curveTo(0.4,83.1,26.8,97.7).lineTo(-29.4,198.9).curveTo(-97.3,161.4,-119.3,86.9).closePath();
	var mask_graphics_423 = new cjs.Graphics().moveTo(-117.6,87.8).curveTo(-139.8,14.3,-104.2,-54.2).curveTo(-68.4,-122.6,4.6,-147).curveTo(77.6,-171.2,147.2,-137.3).lineTo(96.4,-33.2).curveTo(69.3,-46.4,40.9,-37).curveTo(12.4,-27.6,-1.5,-0.9).curveTo(-15.3,25.8,-6.7,54.4).curveTo(2,83.1,28.3,97.7).lineTo(-27.9,198.9).curveTo(-95.2,161.4,-117.6,87.8).closePath();
	var mask_graphics_424 = new cjs.Graphics().moveTo(-115.8,88.5).curveTo(-138.2,15.2,-103.5,-53).curveTo(-68.7,-121.3,3.8,-146.2).curveTo(76.2,-171.1,145.7,-138.7).lineTo(96.7,-33.8).curveTo(69.7,-46.4,41.4,-36.8).curveTo(13.2,-27,-0.3,-0.5).curveTo(-13.8,26.2,-5.2,54.6).curveTo(3.6,83.2,29.8,97.7).lineTo(-26.4,198.9).curveTo(-93.3,161.8,-115.8,88.5).closePath();
	var mask_graphics_425 = new cjs.Graphics().moveTo(-114.1,89.4).curveTo(-136.7,17,-102.6,-51.3).curveTo(-68.5,-119.5,2.9,-145.4).curveTo(74.4,-171.1,144.1,-140.1).lineTo(97.1,-34.3).curveTo(69.9,-46.4,42,-36.4).curveTo(14.3,-26.3,1,0.2).curveTo(-12.3,26.9,-3.5,55.1).curveTo(5.5,83.2,31.3,97.7).lineTo(-24.9,198.9).curveTo(-91.4,161.9,-114.1,89.4).closePath();
	var mask_graphics_426 = new cjs.Graphics().moveTo(-112.2,90.1).curveTo(-135,17.9,-101.8,-50).curveTo(-68.6,-118,2.2,-144.6).curveTo(73,-171,142.6,-141.4).lineTo(97.4,-34.9).curveTo(70.3,-46.4,42.7,-36.1).curveTo(15.1,-25.7,2.2,0.7).curveTo(-10.7,27.3,-1.9,55.3).curveTo(7.1,83.4,32.8,97.7).lineTo(-23.4,198.9).curveTo(-89.5,162.3,-112.2,90.1).closePath();
	var mask_graphics_427 = new cjs.Graphics().moveTo(-110.4,90.9).curveTo(-133.4,19.7,-101.1,-48.3).curveTo(-68.5,-116.3,1.2,-143.6).curveTo(71.1,-171,141.1,-142.7).lineTo(97.8,-35.3).curveTo(70.5,-46.4,43.3,-35.7).curveTo(16.2,-25,3.5,1.4).curveTo(-9.1,27.8,-0.2,55.6).curveTo(8.8,83.4,34.4,97.7).lineTo(-21.8,198.9).curveTo(-87.4,162.3,-110.4,90.9).closePath();
	var mask_graphics_428 = new cjs.Graphics().moveTo(-108.8,91.6).curveTo(-131.8,20.7,-100.2,-47.1).curveTo(-68.7,-114.6,0.5,-142.7).curveTo(69.9,-170.7,139.6,-143.9).lineTo(98,-35.8).curveTo(70.9,-46.2,44,-35.4).curveTo(17,-24.4,4.7,1.8).curveTo(-7.6,28.2,1.3,55.9).curveTo(10.4,83.6,35.9,97.7).lineTo(-20.3,198.9).curveTo(-85.6,162.7,-108.8,91.6).closePath();
	var mask_graphics_429 = new cjs.Graphics().moveTo(-106.9,92.6).curveTo(-130.2,22.3,-99.4,-45.3).curveTo(-68.5,-112.9,0,-141.7).curveTo(68.6,-170.6,138,-145.1).lineTo(98.4,-36.2).curveTo(71.4,-46.1,44.7,-35).curveTo(18.1,-23.8,6,2.5).curveTo(-6.1,28.9,3,56.3).curveTo(12.2,83.6,37.4,97.7).lineTo(-18.8,198.9).curveTo(-83.5,162.8,-106.9,92.6).closePath();
	var mask_graphics_430 = new cjs.Graphics().moveTo(-105.1,93.2).curveTo(-128.4,23.4,-98.5,-44.1).curveTo(-68.5,-111.4,-1,-140.8).curveTo(66.7,-170.2,136.5,-146.2).lineTo(98.7,-36.6).curveTo(71.6,-46,45.2,-34.6).curveTo(18.9,-23.1,7.3,3).curveTo(-4.4,29.3,4.7,56.5).curveTo(13.8,83.8,39,97.7).lineTo(-17.2,198.9).curveTo(-81.7,163.3,-105.1,93.2).closePath();
	var mask_graphics_431 = new cjs.Graphics().moveTo(-103.3,94.1).curveTo(-126.8,25,-97.6,-42.3).curveTo(-68.2,-109.5,-1.3,-139.7).curveTo(65.6,-169.9,134.9,-147.1).lineTo(99,-37).curveTo(72.1,-46,46.1,-34.2).curveTo(20.1,-22.4,8.6,3.7).curveTo(-2.8,30,6.4,56.8).curveTo(15.6,83.8,40.5,97.7).lineTo(-15.7,198.9).curveTo(-79.6,163.3,-103.3,94.1).closePath();
	var mask_graphics_432 = new cjs.Graphics().moveTo(-101.4,94.9).curveTo(-125.1,26.1,-96.7,-41).curveTo(-68.2,-108,-2.3,-138.7).curveTo(63.7,-169.5,133.3,-148.2).lineTo(99.4,-37.4).curveTo(72.4,-45.7,46.6,-33.8).curveTo(21,-21.9,9.9,4.3).curveTo(-1,30.4,8.2,57.1).curveTo(17.4,83.9,42.2,97.7).lineTo(-14,198.9).curveTo(-77.7,163.7,-101.4,94.9).closePath();
	var mask_graphics_433 = new cjs.Graphics().moveTo(-99.6,95.7).curveTo(-123.5,27.7,-95.8,-39.2).curveTo(-67.9,-106.1,-2.7,-137.7).curveTo(62.5,-169.2,131.7,-149.2).lineTo(99.8,-37.8).curveTo(72.8,-45.6,47.4,-33.4).curveTo(22,-21.1,11.2,4.9).curveTo(0.5,30.9,9.7,57.4).curveTo(19.1,83.9,43.7,97.7).lineTo(-12.5,198.9).curveTo(-75.6,163.7,-99.6,95.7).closePath();
	var mask_graphics_434 = new cjs.Graphics().moveTo(-97.7,96.4).curveTo(-121.6,28.6,-94.7,-38).curveTo(-67.7,-104.5,-3.5,-136.6).curveTo(60.8,-168.7,130.1,-150).lineTo(100.2,-38.1).curveTo(73.2,-45.4,48.2,-33).curveTo(23.1,-20.5,12.6,5.4).curveTo(2.1,31.4,11.5,57.8).curveTo(20.8,84.2,45.3,97.7).lineTo(-10.9,198.9).curveTo(-73.7,164.1,-97.7,96.4).closePath();
	var mask_graphics_435 = new cjs.Graphics().moveTo(-95.8,97.3).curveTo(-120.1,30.4,-93.8,-36.1).curveTo(-67.5,-102.6,-4,-135.5).curveTo(59.6,-168.3,128.6,-150.8).lineTo(100.5,-38.5).curveTo(73.7,-45.3,48.9,-32.6).curveTo(24.1,-19.7,13.9,6.2).curveTo(3.7,32,13.2,58).curveTo(22.7,84.2,46.9,97.7).lineTo(-9.3,198.9).curveTo(-71.6,164.2,-95.8,97.3).closePath();
	var mask_graphics_436 = new cjs.Graphics().moveTo(-93.9,98).curveTo(-118.2,31.4,-92.7,-34.9).curveTo(-67.3,-101,-4.7,-134.3).curveTo(57.9,-167.6,126.9,-151.6).lineTo(100.9,-38.8).curveTo(74,-45,49.6,-32.2).curveTo(25.2,-19.2,15.3,6.6).curveTo(5.5,32.4,14.9,58.3).curveTo(24.4,84.3,48.5,97.7).lineTo(-7.7,198.9).curveTo(-69.7,164.6,-93.9,98).closePath();
	var mask_graphics_437 = new cjs.Graphics().moveTo(-92,98.8).curveTo(-116.4,33,-91.6,-33.1).curveTo(-66.9,-99.1,-5.1,-133.1).curveTo(56.8,-167,125.3,-152.3).lineTo(101.2,-39.1).curveTo(74.5,-44.8,50.4,-31.6).curveTo(26.4,-18.3,16.7,7.4).curveTo(7.1,33.1,16.6,58.7).curveTo(26.2,84.3,50.1,97.7).lineTo(-6.1,198.9).curveTo(-67.5,164.6,-92,98.8).closePath();
	var mask_graphics_438 = new cjs.Graphics().moveTo(-90,99.5).curveTo(-114.5,34.1,-90.7,-31.8).curveTo(-66.7,-97.4,-5.9,-132).curveTo(55,-166.4,123.7,-153).lineTo(101.6,-39.3).curveTo(74.9,-44.5,51.2,-31.1).curveTo(27.5,-17.7,18.2,7.9).curveTo(8.8,33.5,18.3,59).curveTo(27.9,84.4,51.8,97.7).lineTo(-4.4,198.9).curveTo(-65.5,165,-90,99.5).closePath();
	var mask_graphics_439 = new cjs.Graphics().moveTo(-88.1,100).curveTo(-112.9,35,-89.6,-30.3).curveTo(-66.3,-95.5,-6.2,-130.6).curveTo(53.9,-165.7,122.1,-153.6).lineTo(102,-39.6).curveTo(75.5,-44.3,52,-30.7).curveTo(28.6,-17,19.5,8.3).curveTo(10.5,33.8,20.1,59.1).curveTo(29.7,84.4,53.4,97.7).lineTo(-2.8,198.9).curveTo(-63.3,165,-88.1,100).closePath();
	var mask_graphics_440 = new cjs.Graphics().moveTo(-86.2,101).curveTo(-111,36.6,-88.5,-28.6).curveTo(-66,-93.9,-6.9,-129.4).curveTo(52.3,-164.9,120.4,-154.2).lineTo(102.3,-39.7).curveTo(75.7,-43.9,52.7,-30.1).curveTo(29.7,-16.3,20.9,9).curveTo(12.2,34.5,21.8,59.5).curveTo(31.5,84.6,55,97.7).lineTo(-1.2,198.9).curveTo(-61.4,165.4,-86.2,101).closePath();
	var mask_graphics_441 = new cjs.Graphics().moveTo(-84.3,101.5).curveTo(-109.2,37.6,-87.4,-27.2).curveTo(-65.5,-91.9,-7.1,-128).curveTo(51.2,-164.2,118.8,-154.6).lineTo(102.7,-40).curveTo(76.4,-43.7,53.7,-29.6).curveTo(30.9,-15.5,22.4,9.7).curveTo(13.9,34.9,23.6,59.8).curveTo(33.4,84.7,56.6,97.7).lineTo(0.4,198.9).curveTo(-59.3,165.6,-84.3,101.5).closePath();
	var mask_graphics_442 = new cjs.Graphics().moveTo(-82.4,102.6).curveTo(-107.3,39.2,-86.4,-25.5).curveTo(-65.2,-90.3,-7.8,-126.8).curveTo(49.6,-163.4,117.2,-155.1).lineTo(103,-40.2).curveTo(76.7,-43.4,54.3,-29.2).curveTo(32,-15,23.7,10.2).curveTo(15.6,35.6,25.4,60.2).curveTo(35.1,84.8,58.3,97.7).lineTo(2.1,198.9).curveTo(-57.4,166,-82.4,102.6).closePath();
	var mask_graphics_443 = new cjs.Graphics().moveTo(-80.3,103.1).curveTo(-105.5,40.3,-85.1,-24).curveTo(-64.6,-88.2,-8,-125.5).curveTo(48.6,-162.6,115.5,-155.4).lineTo(103.4,-40.3).curveTo(77.4,-43,55.4,-28.6).curveTo(33.3,-14.2,25.3,10.9).curveTo(17.3,36,27.1,60.3).curveTo(36.9,84.8,60,97.7).lineTo(3.8,198.9).curveTo(-55.1,166,-80.3,103.1).closePath();
	var mask_graphics_444 = new cjs.Graphics().moveTo(-78.4,104.1).curveTo(-103.6,41.9,-84,-22.3).curveTo(-64.4,-86.5,-8.7,-124.1).curveTo(47,-161.6,113.9,-155.8).lineTo(103.8,-40.4).curveTo(77.7,-42.7,56,-28.1).curveTo(34.4,-13.5,26.7,11.4).curveTo(19.1,36.5,28.8,60.7).curveTo(38.7,85,61.6,97.7).lineTo(5.4,198.9).curveTo(-53.2,166.4,-78.4,104.1).closePath();
	var mask_graphics_445 = new cjs.Graphics().moveTo(-76.5,104.6).curveTo(-101.9,42.9,-82.9,-20.8).curveTo(-63.8,-84.4,-9,-122.6).curveTo(46,-160.7,112.2,-156.1).lineTo(104.1,-40.6).curveTo(78.4,-42.3,57,-27.6).curveTo(35.6,-12.7,28.1,12.1).curveTo(20.8,36.9,30.7,60.9).curveTo(40.6,85,63.2,97.7).lineTo(7,198.9).curveTo(-51.1,166.5,-76.5,104.6).closePath();
	var mask_graphics_446 = new cjs.Graphics().moveTo(-74.6,105.6).curveTo(-99.9,44.5,-81.6,-19.2).curveTo(-63.3,-82.8,-9.4,-121.3).curveTo(44.5,-159.7,110.5,-156.2).lineTo(104.6,-40.6).curveTo(78.7,-41.9,57.7,-27).curveTo(36.9,-12,29.7,12.8).curveTo(22.7,37.6,32.4,61.3).curveTo(42.3,85.1,64.9,97.7).lineTo(8.7,198.9).curveTo(-49.1,166.8,-74.6,105.6).closePath();
	var mask_graphics_447 = new cjs.Graphics().moveTo(-72.5,106.1).curveTo(-98.1,45.4,-80.5,-17.7).curveTo(-62.8,-80.6,-9.7,-119.6).curveTo(43.5,-158.6,108.9,-156.3).lineTo(104.9,-40.7).curveTo(79.4,-41.5,58.7,-26.3).curveTo(38.1,-11.2,31.2,13.3).curveTo(24.3,38,34.2,61.6).curveTo(44.2,85.3,66.5,97.7).lineTo(10.3,198.9).curveTo(-46.9,166.9,-72.5,106.1).closePath();
	var mask_graphics_448 = new cjs.Graphics().moveTo(-70.6,107.2).curveTo(-96,47.1,-79.2,-16).curveTo(-62.3,-79,-10.2,-118.3).curveTo(42,-157.6,107.2,-156.5).lineTo(105.3,-40.7).curveTo(79.9,-41.1,59.6,-25.8).curveTo(39.2,-10.5,32.6,14).curveTo(26.1,38.5,36,62).curveTo(46,85.4,68.2,97.7).lineTo(12,198.9).curveTo(-45,167.3,-70.6,107.2).closePath();
	var mask_graphics_449 = new cjs.Graphics().moveTo(-68.5,107.7).curveTo(-94.3,48.1,-78,-14.4).curveTo(-61.6,-76.9,-10.3,-116.7).curveTo(41,-156.5,105.6,-156.5).lineTo(105.6,-40.7).curveTo(80.4,-40.7,60.4,-25.3).curveTo(40.5,-9.7,34.1,14.6).curveTo(27.7,38.9,37.8,62.1).curveTo(47.8,85.4,69.9,97.7).lineTo(13.7,198.9).curveTo(-42.8,167.3,-68.5,107.7).closePath();
	var mask_graphics_450 = new cjs.Graphics().moveTo(-67.6,108.7).curveTo(-93.2,49.6,-77.6,-12.8).curveTo(-62,-75.2,-11.8,-115.3).curveTo(38.6,-155.3,103,-156.5).lineTo(104.9,-40.7).curveTo(80,-40.3,60.4,-24.7).curveTo(40.7,-9,34.6,15.2).curveTo(28.5,39.6,38.6,62.5).curveTo(48.6,85.5,70.5,97.7).lineTo(14.3,198.9).curveTo(-41.9,167.7,-67.6,108.7).closePath();
	var mask_graphics_451 = new cjs.Graphics().moveTo(-66.6,109.2).curveTo(-92.5,50.7,-77.6,-11.2).curveTo(-62.5,-73.1,-13,-113.7).curveTo(36.7,-154.2,100.3,-156.3).lineTo(104.3,-40.7).curveTo(79.5,-39.7,60.2,-24).curveTo(40.9,-8.2,35.1,15.8).curveTo(29.3,39.9,39.3,62.6).curveTo(49.5,85.5,71.1,97.7).lineTo(14.9,198.9).curveTo(-40.7,167.9,-66.6,109.2).closePath();
	var mask_graphics_452 = new cjs.Graphics().moveTo(-65.6,110.2).curveTo(-91.5,52.2,-77.3,-9.5).curveTo(-62.9,-71.3,-14.3,-112.2).curveTo(34.3,-153,97.7,-156.2).lineTo(103.6,-40.6).curveTo(79,-39.3,60,-23.5).curveTo(41.2,-7.5,35.7,16.5).curveTo(30.1,40.6,40.1,63).curveTo(50.1,85.7,71.8,97.7).lineTo(15.6,198.9).curveTo(-39.8,168.1,-65.6,110.2).closePath();
	var mask_graphics_453 = new cjs.Graphics().moveTo(-64.6,110.7).curveTo(-90.6,53.3,-76.9,-7.9).curveTo(-63.3,-69.1,-15.5,-110.4).curveTo(32.3,-151.6,94.9,-156.1).lineTo(103,-40.6).curveTo(78.7,-38.8,60,-22.8).curveTo(41.4,-6.7,36.1,17.1).curveTo(30.9,41,40.9,63.3).curveTo(51,85.7,72.4,97.7).lineTo(16.2,198.9).curveTo(-38.6,168.3,-64.6,110.7).closePath();
	var mask_graphics_454 = new cjs.Graphics().moveTo(-63.7,111.7).curveTo(-89.5,54.8,-76.5,-6.4).curveTo(-63.5,-67.5,-16.8,-108.9).curveTo(30,-150.3,92.3,-155.8).lineTo(102.4,-40.4).curveTo(78.1,-38.3,59.8,-22.1).curveTo(41.7,-6,36.7,17.7).curveTo(31.7,41.5,41.7,63.7).curveTo(51.8,85.9,73.1,97.7).lineTo(16.9,198.9).curveTo(-37.8,168.7,-63.7,111.7).closePath();
	var mask_graphics_455 = new cjs.Graphics().moveTo(-62.7,112.2).curveTo(-88.8,55.9,-76.3,-4.7).curveTo(-63.9,-65.2,-17.8,-107.1).curveTo(28.2,-148.9,89.7,-155.4).lineTo(101.7,-40.3).curveTo(77.8,-37.7,59.9,-21.5).curveTo(42,-5.2,37.1,18.4).curveTo(32.3,41.9,42.4,63.9).curveTo(52.6,85.9,73.7,97.7).lineTo(17.5,198.9).curveTo(-36.5,168.7,-62.7,112.2).closePath();
	var mask_graphics_456 = new cjs.Graphics().moveTo(-61.8,113.1).curveTo(-87.8,57.4,-76,-3.2).curveTo(-64.2,-63.6,-19.3,-105.6).curveTo(25.8,-147.5,86.9,-155.1).lineTo(101.1,-40.2).curveTo(77.3,-37.2,59.7,-20.9).curveTo(42.2,-4.5,37.6,19).curveTo(33,42.6,43.2,64.3).curveTo(53.3,86.1,74.3,97.7).lineTo(18.1,198.9).curveTo(-35.8,169.1,-61.8,113.1).closePath();
	var mask_graphics_457 = new cjs.Graphics().moveTo(-60.7,113.7).curveTo(-86.8,58.4,-75.7,-1.4).curveTo(-64.5,-61.3,-20.2,-103.7).curveTo(24.1,-146.1,84.3,-154.6).lineTo(100.5,-40).curveTo(77,-36.6,59.7,-20.1).curveTo(42.5,-3.6,38.2,19.7).curveTo(33.8,43,44,64.5).curveTo(54.1,86.1,75,97.7).lineTo(18.8,198.9).curveTo(-34.6,169.1,-60.7,113.7).closePath();
	var mask_graphics_458 = new cjs.Graphics().moveTo(-59.8,114.6).curveTo(-86,59.9,-75.4,0.2).curveTo(-64.8,-59.5,-21.6,-102.2).curveTo(21.7,-144.7,81.7,-154.2).lineTo(99.8,-39.7).curveTo(76.4,-36.1,59.6,-19.6).curveTo(42.8,-2.9,38.6,20.2).curveTo(34.6,43.5,44.7,64.8).curveTo(54.9,86.2,75.6,97.7).lineTo(19.4,198.9).curveTo(-33.7,169.5,-59.8,114.6).closePath();
	var mask_graphics_459 = new cjs.Graphics().moveTo(-58.7,115.2).curveTo(-85,60.9,-75,1.7).curveTo(-65,-57.4,-22.4,-100.3).curveTo(20.1,-143.1,79.1,-153.6).lineTo(99.2,-39.6).curveTo(76.3,-35.4,59.6,-18.8).curveTo(43.1,-2.1,39.2,20.9).curveTo(35.4,43.9,45.5,65.1).curveTo(55.8,86.2,76.3,97.7).lineTo(20.1,198.9).curveTo(-32.5,169.6,-58.7,115.2).closePath();
	var mask_graphics_460 = new cjs.Graphics().moveTo(-57.9,116.1).curveTo(-84,62.5,-74.6,3.5).curveTo(-65.3,-55.6,-23.7,-98.7).curveTo(17.9,-141.6,76.5,-153).lineTo(98.6,-39.3).curveTo(75.7,-34.9,59.6,-18.2).curveTo(43.4,-1.4,39.8,21.5).curveTo(36.1,44.5,46.3,65.3).curveTo(56.4,86.3,76.9,97.7).lineTo(20.7,198.9).curveTo(-31.6,169.9,-57.9,116.1).closePath();
	var mask_graphics_461 = new cjs.Graphics().moveTo(-56.8,116.7).curveTo(-83.1,63.4,-74.3,5.1).curveTo(-65.5,-53.3,-24.7,-96.6).curveTo(16,-140,73.8,-152.3).lineTo(98,-39.1).curveTo(75.5,-34.2,59.5,-17.4).curveTo(43.6,-0.5,40.3,22.1).curveTo(36.9,44.9,47,65.6).curveTo(57.3,86.5,77.5,97.7).lineTo(21.3,198.9).curveTo(-30.4,170,-56.8,116.7).closePath();
	var mask_graphics_462 = new cjs.Graphics().moveTo(-55.8,117.8).curveTo(-82,65.1,-73.9,6.7).curveTo(-65.6,-51.5,-25.8,-95).curveTo(14,-138.3,71.3,-151.6).lineTo(97.3,-38.8).curveTo(75.1,-33.7,59.5,-16.7).curveTo(44.1,0.2,40.8,22.8).curveTo(37.7,45.6,47.8,66).curveTo(58.1,86.6,78.2,97.7).lineTo(22,198.9).curveTo(-29.5,170.4,-55.8,117.8).closePath();
	var mask_graphics_463 = new cjs.Graphics().moveTo(-54.7,118.2).curveTo(-81.1,66,-73.4,8.3).curveTo(-65.7,-49.4,-26.8,-93).curveTo(12.2,-136.6,68.6,-150.8).lineTo(96.7,-38.5).curveTo(74.7,-33,59.6,-16).curveTo(44.4,1,41.4,23.4).curveTo(38.4,45.8,48.6,66.2).curveTo(58.9,86.6,78.8,97.7).lineTo(22.6,198.9).curveTo(-28.3,170.4,-54.7,118.2).closePath();
	var mask_graphics_464 = new cjs.Graphics().moveTo(-53.9,119.1).curveTo(-80.1,67.5,-73.1,10).curveTo(-65.9,-47.5,-28,-91.3).curveTo(10.1,-135.1,66.1,-150).lineTo(96.1,-38.1).curveTo(74.3,-32.3,59.4,-15.4).curveTo(44.6,1.7,41.9,24).curveTo(39.2,46.5,49.3,66.6).curveTo(59.6,86.7,79.4,97.7).lineTo(23.2,198.9).curveTo(-27.4,170.8,-53.9,119.1).closePath();
	var mask_graphics_465 = new cjs.Graphics().moveTo(-66.5,85.3).curveTo(-88.2,9.8,-50.1,-58.8).curveTo(-11.9,-127.5,63.5,-149.2).lineTo(95.4,-37.8).curveTo(66.1,-29.5,51.2,-2.8).curveTo(36.4,24,44.8,53.4).curveTo(53.2,82.8,80,97.7).lineTo(23.8,198.9).curveTo(-44.9,160.8,-66.5,85.3).closePath();
	var mask_graphics_466 = new cjs.Graphics().moveTo(-65.8,86.6).curveTo(-87.7,12.4,-50.6,-56.5).curveTo(-13.5,-125.3,61,-148.2).lineTo(94.8,-37.4).curveTo(65.9,-28.5,51.4,-1.8).curveTo(37,25,45.5,53.8).curveTo(54.1,82.8,80.6,97.7).lineTo(24.4,198.9).curveTo(-43.7,161,-65.8,86.6).closePath();
	var mask_graphics_467 = new cjs.Graphics().moveTo(-64.6,87.7).curveTo(-86.8,14.2,-50.8,-54.6).curveTo(-14.8,-123.3,58.3,-147.1).lineTo(94.2,-37).curveTo(65.7,-27.7,51.6,-1).curveTo(37.6,25.7,46.3,54.4).curveTo(55,83.1,81.2,97.7).lineTo(25,198.9).curveTo(-42.3,161.4,-64.6,87.7).closePath();
	var mask_graphics_468 = new cjs.Graphics().moveTo(-63.7,88.5).curveTo(-86.1,15.2,-51.4,-53).curveTo(-16.6,-121.3,55.8,-146.2).lineTo(93.6,-36.6).curveTo(65.3,-27,51.8,-0.5).curveTo(38.2,26.2,46.9,54.6).curveTo(55.7,83.2,81.8,97.7).lineTo(25.6,198.9).curveTo(-41.3,161.8,-63.7,88.5).closePath();
	var mask_graphics_469 = new cjs.Graphics().moveTo(-62.9,89.9).curveTo(-85.6,17.8,-51.9,-50.6).curveTo(-18,-119,53.3,-145.1).lineTo(93,-36.2).curveTo(65.2,-26.1,52,0.5).curveTo(38.8,27.2,47.6,55.2).curveTo(56.6,83.2,82.4,97.7).lineTo(26.2,198.9).curveTo(-40.1,161.9,-62.9,89.9).closePath();
	var mask_graphics_470 = new cjs.Graphics().moveTo(-61.8,90.9).curveTo(-84.7,19.6,-52,-48.7).curveTo(-19.2,-116.8,50.8,-143.9).lineTo(92.4,-35.8).curveTo(65,-25.3,52.3,1.3).curveTo(39.6,27.8,48.5,55.6).curveTo(57.5,83.4,83.1,97.7).lineTo(26.9,198.9).curveTo(-38.8,162.3,-61.8,90.9).closePath();
	var mask_graphics_471 = new cjs.Graphics().moveTo(-61,91.6).curveTo(-84,20.7,-52.4,-47.1).curveTo(-20.9,-114.6,48.5,-142.7).lineTo(91.8,-35.3).curveTo(64.8,-24.4,52.5,1.8).curveTo(40.2,28.2,49.1,55.9).curveTo(58.2,83.6,83.7,97.7).lineTo(27.5,198.9).curveTo(-37.8,162.7,-61,91.6).closePath();
	var mask_graphics_472 = new cjs.Graphics().moveTo(-60.1,93).curveTo(-83.4,23.1,-52.8,-44.6).curveTo(-22.2,-112.3,46,-141.4).lineTo(91.2,-34.9).curveTo(64.6,-23.5,52.7,2.8).curveTo(40.8,29.2,49.9,56.4).curveTo(59.1,83.6,84.3,97.7).lineTo(28.1,198.9).curveTo(-36.7,162.8,-60.1,93).closePath();
	var mask_graphics_473 = new cjs.Graphics().moveTo(-58.9,94.1).curveTo(-82.5,24.9,-53,-42.7).curveTo(-23.3,-110.2,43.6,-140.1).lineTo(90.6,-34.3).curveTo(64.6,-22.7,53.1,3.6).curveTo(41.5,29.9,50.8,56.8).curveTo(60,83.8,84.9,97.7).lineTo(28.7,198.9).curveTo(-35.2,163.3,-58.9,94.1).closePath();
	var mask_graphics_474 = new cjs.Graphics().moveTo(-58.1,94.9).curveTo(-81.8,26.1,-53.4,-41).curveTo(-24.9,-108,41,-138.7).lineTo(90,-33.8).curveTo(64.3,-21.9,53.2,4.3).curveTo(42.2,30.4,51.4,57.1).curveTo(60.6,83.9,85.4,97.7).lineTo(29.2,198.9).curveTo(-34.4,163.7,-58.1,94.9).closePath();
	var mask_graphics_475 = new cjs.Graphics().moveTo(-57.2,96.1).curveTo(-81.2,28.5,-53.7,-38.5).curveTo(-26.1,-105.4,38.6,-137.3).lineTo(89.4,-33.2).curveTo(64.2,-20.8,53.5,5.2).curveTo(42.8,31.4,52,57.6).curveTo(61.4,83.9,86,97.7).lineTo(29.8,198.9).curveTo(-33.3,163.7,-57.2,96.1).closePath();
	var mask_graphics_476 = new cjs.Graphics().moveTo(-56.2,97.2).curveTo(-80.4,30.3,-53.9,-36.5).curveTo(-27.2,-103.3,36.2,-135.8).lineTo(88.9,-32.7).curveTo(64.1,-20,53.7,6).curveTo(43.4,32,52.9,58).curveTo(62.3,84.2,86.6,97.7).lineTo(30.4,198.9).curveTo(-31.9,164.2,-56.2,97.2).closePath();
	var mask_graphics_477 = new cjs.Graphics().moveTo(-55.3,98).curveTo(-79.5,31.4,-54.1,-34.9).curveTo(-28.6,-101,34,-134.3).lineTo(88.3,-32).curveTo(63.9,-19.2,54,6.6).curveTo(44.1,32.4,53.6,58.3).curveTo(63.1,84.3,87.2,97.7).lineTo(31,198.9).curveTo(-31,164.6,-55.3,98).closePath();
	var mask_graphics_478 = new cjs.Graphics().moveTo(-54.3,98.8).curveTo(-78.8,33.1,-54.3,-32.7).curveTo(-29.8,-98.4,31.5,-132.6).lineTo(87.7,-31.5).curveTo(63.8,-18.1,54.1,7.5).curveTo(44.7,33.1,54.1,58.7).curveTo(63.8,84.3,87.7,97.7).lineTo(31.5,198.9).curveTo(-29.8,164.6,-54.3,98.8).closePath();
	var mask_graphics_479 = new cjs.Graphics().moveTo(-53.9,99.9).curveTo(-78.6,34.9,-55,-30.7).curveTo(-31.4,-96.1,28.7,-131).lineTo(86.6,-30.8).curveTo(63.2,-17.3,54,8.2).curveTo(44.8,33.8,54.4,59.1).curveTo(64,84.4,87.7,97.7).lineTo(31.5,198.9).curveTo(-29.1,165,-53.9,99.9).closePath();
	var mask_graphics_480 = new cjs.Graphics().moveTo(-53.5,101).curveTo(-78.3,36.6,-55.8,-28.6).curveTo(-33.3,-93.9,25.8,-129.4).lineTo(85.4,-30.1).curveTo(62.4,-16.3,53.6,9).curveTo(44.9,34.5,54.6,59.5).curveTo(64.2,84.6,87.7,97.7).lineTo(31.5,198.9).curveTo(-28.7,165.4,-53.5,101).closePath();
	var mask_graphics_481 = new cjs.Graphics().moveTo(-53.2,101.9).curveTo(-78.2,38.4,-56.6,-26.5).curveTo(-35,-91.2,23,-127.6).lineTo(84.3,-29.5).curveTo(61.9,-15.2,53.3,10).curveTo(44.9,35.1,54.7,59.9).curveTo(64.4,84.7,87.7,97.7).lineTo(31.5,198.9).curveTo(-28.2,165.6,-53.2,101.9).closePath();
	var mask_graphics_482 = new cjs.Graphics().moveTo(-52.7,103).curveTo(-77.8,40.2,-57.2,-24.4).curveTo(-36.5,-88.9,20.3,-125.9).lineTo(83.3,-28.8).curveTo(61.2,-14.4,53.1,10.6).curveTo(45.1,35.8,54.8,60.3).curveTo(64.7,84.8,87.7,97.7).lineTo(31.5,198.9).curveTo(-27.5,166,-52.7,103).closePath();
	var mask_graphics_483 = new cjs.Graphics().moveTo(-52.3,104.1).curveTo(-77.5,41.9,-57.9,-22.3).curveTo(-38.2,-86.5,17.4,-124.1).lineTo(82.2,-28.1).curveTo(60.5,-13.5,52.8,11.4).curveTo(45.2,36.5,55,60.7).curveTo(64.8,85,87.7,97.7).lineTo(31.5,198.9).curveTo(-27.1,166.4,-52.3,104.1).closePath();
	var mask_graphics_484 = new cjs.Graphics().moveTo(-52,105).curveTo(-77.4,43.7,-58.5,-20.1).curveTo(-39.7,-83.8,14.7,-122.2).lineTo(81.1,-27.3).curveTo(60,-12.4,52.7,12.4).curveTo(45.3,37.2,55.2,61).curveTo(65.1,85,87.7,97.7).lineTo(31.5,198.9).curveTo(-26.6,166.5,-52,105).closePath();
	var mask_graphics_485 = new cjs.Graphics().moveTo(-51.5,106.1).curveTo(-76.9,45.4,-59.1,-18.1).curveTo(-41.1,-81.5,12,-120.3).lineTo(80.1,-26.6).curveTo(59.4,-11.4,52.4,13.2).curveTo(45.5,37.9,55.4,61.6).curveTo(65.4,85.3,87.7,97.7).lineTo(31.5,198.9).curveTo(-25.9,166.9,-51.5,106.1).closePath();
	var mask_graphics_486 = new cjs.Graphics().moveTo(-51.1,107.2).curveTo(-76.5,47.1,-59.7,-16).curveTo(-42.8,-79,9.3,-118.3).lineTo(79.1,-25.8).curveTo(58.7,-10.5,52.1,14).curveTo(45.6,38.5,55.5,62).curveTo(65.5,85.4,87.7,97.7).lineTo(31.5,198.9).curveTo(-25.5,167.3,-51.1,107.2).closePath();
	var mask_graphics_487 = new cjs.Graphics().moveTo(-50.7,108).curveTo(-76.3,48.8,-60.3,-13.7).curveTo(-44.2,-76.2,6.7,-116.3).lineTo(78,-25).curveTo(58.2,-9.4,52,14.8).curveTo(45.7,39.2,55.6,62.2).curveTo(65.7,85.4,87.7,97.7).lineTo(31.5,198.9).curveTo(-24.9,167.3,-50.7,108).closePath();
	var mask_graphics_488 = new cjs.Graphics().moveTo(-50.1,109.1).curveTo(-75.9,50.6,-60.7,-11.6).curveTo(-45.4,-73.7,4.2,-114.2).lineTo(77,-24.3).curveTo(57.7,-8.5,51.7,15.6).curveTo(45.9,39.9,55.9,62.6).curveTo(65.9,85.5,87.7,97.7).lineTo(31.5,198.9).curveTo(-24.3,167.7,-50.1,109.1).closePath();
	var mask_graphics_489 = new cjs.Graphics().moveTo(-49.7,110.2).curveTo(-75.6,52.2,-61.4,-9.5).curveTo(-47,-71.3,1.6,-112.1).lineTo(76.1,-23.4).curveTo(57.1,-7.5,51.6,16.5).curveTo(46,40.6,56,63).curveTo(66.1,85.7,87.7,97.7).lineTo(31.5,198.9).curveTo(-23.9,168.1,-49.7,110.2).closePath();
	var mask_graphics_490 = new cjs.Graphics().moveTo(-49.3,111.1).curveTo(-75.2,54,-61.8,-7.2).curveTo(-48.4,-68.5,-0.8,-110).lineTo(75,-22.5).curveTo(56.6,-6.4,51.3,17.4).curveTo(46.2,41.2,56.2,63.4).curveTo(66.3,85.7,87.7,97.7).lineTo(31.5,198.9).curveTo(-23.3,168.3,-49.3,111.1).closePath();
	var mask_graphics_491 = new cjs.Graphics().moveTo(-48.8,112.2).curveTo(-74.8,55.7,-62.2,-5.2).curveTo(-49.5,-66,-3.4,-107.7).lineTo(74.1,-21.7).curveTo(56.2,-5.5,51.2,18.2).curveTo(46.3,41.9,56.4,63.9).curveTo(66.6,85.9,87.7,97.7).lineTo(31.5,198.9).curveTo(-22.6,168.7,-48.8,112.2).closePath();
	var mask_graphics_492 = new cjs.Graphics().moveTo(-48.4,113.1).curveTo(-74.4,57.4,-62.6,-3.2).curveTo(-50.8,-63.6,-5.8,-105.6).lineTo(73.1,-20.9).curveTo(55.6,-4.5,51,19).curveTo(46.4,42.6,56.6,64.3).curveTo(66.7,86.1,87.7,97.7).lineTo(31.5,198.9).curveTo(-22.4,169.1,-48.4,113.1).closePath();
	var mask_graphics_493 = new cjs.Graphics().moveTo(-47.8,114.1).curveTo(-74,59.1,-63.1,-0.7).curveTo(-52.2,-60.6,-8.3,-103.3).lineTo(72.2,-20).curveTo(55.1,-3.3,50.8,20).curveTo(46.6,43.3,56.7,64.7).curveTo(67,86.1,87.7,97.7).lineTo(31.5,198.9).curveTo(-21.7,169.1,-47.8,114.1).closePath();
	var mask_graphics_494 = new cjs.Graphics().moveTo(-47.3,115.2).curveTo(-73.6,60.9,-63.4,1.4).curveTo(-53.1,-58,-10.6,-101).lineTo(71.3,-19).curveTo(54.7,-2.4,50.8,20.8).curveTo(46.8,43.9,57,65.1).curveTo(67.3,86.2,87.7,97.7).lineTo(31.5,198.9).curveTo(-21,169.5,-47.3,115.2).closePath();
	var mask_graphics_495 = new cjs.Graphics().moveTo(-47,116.1).curveTo(-73.2,62.5,-63.8,3.5).curveTo(-54.5,-55.6,-12.9,-98.7).lineTo(70.4,-18.2).curveTo(54.3,-1.4,50.6,21.5).curveTo(47,44.5,57.1,65.3).curveTo(67.3,86.3,87.7,97.7).lineTo(31.5,198.9).curveTo(-20.7,169.9,-47,116.1).closePath();
	var mask_graphics_496 = new cjs.Graphics().moveTo(-46.5,117.1).curveTo(-72.7,64.3,-64.2,5.8).curveTo(-55.5,-52.6,-15.2,-96.2).lineTo(69.4,-17.3).curveTo(53.7,-0.2,50.4,22.4).curveTo(47.1,45.2,57.3,65.8).curveTo(67.6,86.5,87.7,97.7).lineTo(31.5,198.9).curveTo(-20.2,170,-46.5,117.1).closePath();
	var mask_graphics_497 = new cjs.Graphics().moveTo(-45.8,118.2).curveTo(-72.2,65.9,-64.4,7.9).curveTo(-56.5,-50,-17.4,-93.8).lineTo(68.6,-16.3).curveTo(53.5,0.7,50.4,23.2).curveTo(47.4,45.8,57.5,66.2).curveTo(67.8,86.6,87.7,97.7).lineTo(31.5,198.9).curveTo(-19.4,170.4,-45.8,118.2).closePath();
	var mask_graphics_498 = new cjs.Graphics().moveTo(-45.5,119.1).curveTo(-71.8,67.5,-64.8,10).curveTo(-57.6,-47.5,-19.7,-91.2).lineTo(67.8,-15.4).curveTo(52.9,1.7,50.2,24).curveTo(47.5,46.5,57.7,66.6).curveTo(68,86.7,87.7,97.7).lineTo(31.5,198.9).curveTo(-19.1,170.8,-45.5,119.1).closePath();
	var mask_graphics_499 = new cjs.Graphics().moveTo(-45,120.1).curveTo(-71.3,69.3,-65,12.4).curveTo(-58.7,-44.5,-21.7,-88.8).lineTo(67,-14.3).curveTo(52.5,2.9,50.1,25).curveTo(47.6,47.2,57.9,67).curveTo(68.2,86.7,87.7,97.7).lineTo(31.5,198.9).curveTo(-18.6,171,-45,120.1).closePath();
	var mask_graphics_500 = new cjs.Graphics().moveTo(-44.3,121).curveTo(-70.7,70.9,-65.2,14.4).curveTo(-59.5,-41.9,-23.9,-86.2).lineTo(66.1,-13.3).curveTo(52.2,3.9,50.1,25.8).curveTo(47.9,47.9,58.2,67.4).curveTo(68.5,86.9,87.7,97.7).lineTo(31.5,198.9).curveTo(-17.8,171.2,-44.3,121).closePath();
	var mask_graphics_501 = new cjs.Graphics().moveTo(-43.9,122.1).curveTo(-70.3,72.5,-65.4,16.6).curveTo(-60.6,-39.3,-25.9,-83.6).lineTo(65.4,-12.4).curveTo(51.8,4.9,49.9,26.6).curveTo(48.1,48.4,58.3,67.6).curveTo(68.6,87,87.7,97.7).lineTo(31.5,198.9).curveTo(-17.5,171.7,-43.9,122.1).closePath();
	var mask_graphics_502 = new cjs.Graphics().moveTo(-43.4,123).curveTo(-69.8,74.3,-65.6,19).curveTo(-61.4,-36.2,-27.9,-81).lineTo(64.6,-11.3).curveTo(51.6,6.2,49.9,27.6).curveTo(48.3,49.1,58.5,68.1).curveTo(68.8,87.1,87.7,97.7).lineTo(31.5,198.9).curveTo(-17,171.8,-43.4,123).closePath();
	var mask_graphics_503 = new cjs.Graphics().moveTo(-42.8,124).curveTo(-69.2,75.9,-65.7,21.1).curveTo(-62.2,-33.7,-30,-78.3).lineTo(63.8,-10.2).curveTo(51.2,7.1,49.8,28.4).curveTo(48.5,49.8,58.7,68.5).curveTo(69,87.3,87.7,97.7).lineTo(31.5,198.9).curveTo(-16.3,172.2,-42.8,124).closePath();
	var mask_graphics_504 = new cjs.Graphics().moveTo(-42.4,125.1).curveTo(-68.7,77.5,-65.8,23.2).curveTo(-63,-31.1,-31.8,-75.6).lineTo(63.1,-9.3).curveTo(50.9,8.1,49.8,29.2).curveTo(48.7,50.4,58.9,68.9).curveTo(69.2,87.4,87.7,97.7).lineTo(31.5,198.9).curveTo(-16,172.6,-42.4,125.1).closePath();
	var mask_graphics_505 = new cjs.Graphics().moveTo(-41.7,125.9).curveTo(-68.1,79.2,-66,25.5).curveTo(-63.8,-28,-33.7,-72.9).lineTo(62.3,-8.2).curveTo(50.6,9.3,49.7,30.1).curveTo(48.9,51,59.2,69.1).curveTo(69.4,87.4,87.7,97.7).lineTo(31.5,198.9).curveTo(-15.3,172.6,-41.7,125.9).closePath();
	var mask_graphics_506 = new cjs.Graphics().moveTo(-41.1,126.8).curveTo(-67.5,80.8,-66,27.7).curveTo(-64.4,-25.4,-35.5,-70.1).lineTo(61.6,-7.1).curveTo(50.4,10.4,49.7,30.9).curveTo(49.1,51.7,59.4,69.5).curveTo(69.7,87.6,87.7,97.7).lineTo(31.5,198.9).curveTo(-14.7,173,-41.1,126.8).closePath();
	var mask_graphics_507 = new cjs.Graphics().moveTo(-40.8,127.9).curveTo(-67.1,82.4,-66.1,29.7).curveTo(-65.2,-22.8,-37.3,-67.4).lineTo(60.9,-6).curveTo(50.1,11.3,49.7,31.8).curveTo(49.3,52.3,59.6,69.9).curveTo(69.9,87.7,87.7,97.7).lineTo(31.5,198.9).curveTo(-14.4,173.4,-40.8,127.9).closePath();
	var mask_graphics_508 = new cjs.Graphics().moveTo(-40.1,128.7).curveTo(-66.4,84,-66.1,32.2).curveTo(-65.8,-19.6,-39,-64.5).lineTo(60.2,-4.9).curveTo(49.8,12.5,49.7,32.7).curveTo(49.5,52.9,59.7,70.4).curveTo(70,87.8,87.7,97.7).lineTo(31.5,198.9).curveTo(-13.8,173.5,-40.1,128.7).closePath();
	var mask_graphics_509 = new cjs.Graphics().moveTo(-39.4,129.8).curveTo(-65.7,85.7,-66.1,34.3).curveTo(-66.4,-17,-40.7,-61.7).lineTo(59.6,-3.7).curveTo(49.5,13.6,49.7,33.5).curveTo(49.8,53.6,60,70.8).curveTo(70.3,88,87.7,97.7).lineTo(31.5,198.9).curveTo(-13.2,174,-39.4,129.8).closePath();
	var mask_graphics_510 = new cjs.Graphics().moveTo(-58.8,85.3).curveTo(-80.5,9.8,-42.3,-58.8).lineTo(58.9,-2.6).curveTo(44.1,24,52.5,53.4).curveTo(60.9,82.8,87.7,97.7).lineTo(31.5,198.9).curveTo(-37.1,160.8,-58.8,85.3).closePath();
	var mask_graphics_511 = new cjs.Graphics().moveTo(-58.4,86.9).curveTo(-80.3,12.5,-43.9,-55.9).lineTo(58.3,-1.6).curveTo(44.1,25.1,52.7,54.1).curveTo(61.3,83.1,87.7,97.7).lineTo(31.5,198.9).curveTo(-36.3,161.4,-58.4,86.9).closePath();
	var mask_graphics_512 = new cjs.Graphics().moveTo(-57.9,88.5).curveTo(-80.2,15.2,-45.4,-53).lineTo(57.7,-0.3).curveTo(44.1,26.2,52.8,54.6).curveTo(61.6,83.2,87.7,97.7).lineTo(31.5,198.9).curveTo(-35.4,161.8,-57.9,88.5).closePath();
	var mask_graphics_513 = new cjs.Graphics().moveTo(-57.3,90.1).curveTo(-80.1,17.9,-46.9,-50).lineTo(57.1,0.7).curveTo(44.3,27.3,53.1,55.3).curveTo(62,83.4,87.7,97.7).lineTo(31.5,198.9).curveTo(-34.6,162.3,-57.3,90.1).closePath();
	var mask_graphics_514 = new cjs.Graphics().moveTo(-56.9,91.6).curveTo(-79.9,20.7,-48.4,-47.1).lineTo(56.6,2).curveTo(44.3,28.2,53.2,55.9).curveTo(62.3,83.6,87.7,97.7).lineTo(31.5,198.9).curveTo(-33.7,162.7,-56.9,91.6).closePath();
	var mask_graphics_515 = new cjs.Graphics().moveTo(-56.4,93.2).curveTo(-79.7,23.4,-49.7,-43.9).lineTo(56,3).curveTo(44.4,29.3,53.5,56.5).curveTo(62.5,83.8,87.7,97.7).lineTo(31.5,198.9).curveTo(-32.9,163.3,-56.4,93.2).closePath();
	var mask_graphics_516 = new cjs.Graphics().moveTo(-55.8,94.9).curveTo(-79.5,26.1,-51.1,-41).lineTo(55.5,4.3).curveTo(44.5,30.4,53.7,57.1).curveTo(62.9,83.9,87.7,97.7).lineTo(31.5,198.9).curveTo(-32.1,163.7,-55.8,94.9).closePath();
	var mask_graphics_517 = new cjs.Graphics().moveTo(-55.3,96.4).curveTo(-79.2,28.6,-52.3,-37.8).lineTo(55.1,5.5).curveTo(44.5,31.4,53.9,57.8).curveTo(63.2,84.2,87.7,97.7).lineTo(31.5,198.9).curveTo(-31.3,164.1,-55.3,96.4).closePath();
	var mask_graphics_518 = new cjs.Graphics().moveTo(-54.7,98).curveTo(-79,31.4,-53.5,-34.9).lineTo(54.6,6.7).curveTo(44.7,32.4,54.1,58.3).curveTo(63.6,84.3,87.7,97.7).lineTo(31.5,198.9).curveTo(-30.5,164.6,-54.7,98).closePath();
	var mask_graphics_519 = new cjs.Graphics().moveTo(-54.1,99.5).curveTo(-78.6,34.1,-54.7,-31.8).lineTo(54.1,7.9).curveTo(44.8,33.5,54.3,59).curveTo(63.9,84.4,87.7,97.7).lineTo(31.5,198.9).curveTo(-29.5,165,-54.1,99.5).closePath();
	var mask_graphics_520 = new cjs.Graphics().moveTo(-53.5,101).curveTo(-78.3,36.6,-55.8,-28.6).lineTo(53.7,9.1).curveTo(44.9,34.5,54.6,59.5).curveTo(64.2,84.6,87.7,97.7).lineTo(31.5,198.9).curveTo(-28.7,165.4,-53.5,101).closePath();
	var mask_graphics_521 = new cjs.Graphics().moveTo(-53,102.6).curveTo(-77.9,39.2,-56.8,-25.5).lineTo(53.3,10.4).curveTo(45.1,35.6,54.8,60.2).curveTo(64.6,84.8,87.7,97.7).lineTo(31.5,198.9).curveTo(-27.9,166,-53,102.6).closePath();
	var mask_graphics_522 = new cjs.Graphics().moveTo(-52.3,104.1).curveTo(-77.5,41.9,-57.9,-22.3).lineTo(52.9,11.6).curveTo(45.2,36.5,55,60.7).curveTo(64.8,85,87.7,97.7).lineTo(31.5,198.9).curveTo(-27.1,166.4,-52.3,104.1).closePath();
	var mask_graphics_523 = new cjs.Graphics().moveTo(-51.8,105.6).curveTo(-77.1,44.5,-58.8,-19.2).lineTo(52.5,12.8).curveTo(45.5,37.6,55.2,61.3).curveTo(65.1,85.1,87.7,97.7).lineTo(31.5,198.9).curveTo(-26.3,166.8,-51.8,105.6).closePath();
	var mask_graphics_524 = new cjs.Graphics().moveTo(-51.1,107.2).curveTo(-76.5,47.1,-59.6,-15.9).lineTo(52.2,14).curveTo(45.6,38.5,55.5,62).curveTo(65.5,85.4,87.7,97.7).lineTo(31.5,198.9).curveTo(-25.5,167.3,-51.1,107.2).closePath();
	var mask_graphics_525 = new cjs.Graphics().moveTo(-50.4,108.7).curveTo(-76,49.6,-60.4,-12.8).lineTo(51.8,15.2).curveTo(45.7,39.6,55.8,62.5).curveTo(65.8,85.5,87.7,97.7).lineTo(31.5,198.9).curveTo(-24.7,167.7,-50.4,108.7).closePath();
	var mask_graphics_526 = new cjs.Graphics().moveTo(-49.7,110.2).curveTo(-75.6,52.2,-61.2,-9.5).lineTo(51.6,16.5).curveTo(46,40.6,56,63).curveTo(66.1,85.7,87.7,97.7).lineTo(31.5,198.9).curveTo(-23.9,168.1,-49.7,110.2).closePath();
	var mask_graphics_527 = new cjs.Graphics().moveTo(-49,111.7).curveTo(-74.9,54.8,-61.9,-6.3).lineTo(51.3,17.8).curveTo(46.3,41.5,56.3,63.7).curveTo(66.5,85.9,87.7,97.7).lineTo(31.5,198.9).curveTo(-23.2,168.7,-49,111.7).closePath();
	var mask_graphics_528 = new cjs.Graphics().moveTo(-48.4,113.1).curveTo(-74.4,57.4,-62.6,-3).lineTo(51,19).curveTo(46.4,42.6,56.6,64.3).curveTo(66.7,86.1,87.7,97.7).lineTo(31.5,198.9).curveTo(-22.4,169.1,-48.4,113.1).closePath();
	var mask_graphics_529 = new cjs.Graphics().moveTo(-47.7,114.6).curveTo(-73.8,59.9,-63.3,0.2).lineTo(50.8,20.2).curveTo(46.7,43.5,56.9,64.8).curveTo(67,86.2,87.7,97.7).lineTo(31.5,198.9).curveTo(-21.6,169.5,-47.7,114.6).closePath();
	var mask_graphics_530 = new cjs.Graphics().moveTo(-47,116.1).curveTo(-73.2,62.5,-63.8,3.5).lineTo(50.6,21.6).curveTo(47,44.5,57.1,65.3).curveTo(67.3,86.3,87.7,97.7).lineTo(31.5,198.9).curveTo(-20.7,169.9,-47,116.1).closePath();
	var mask_graphics_531 = new cjs.Graphics().moveTo(-46.2,117.8).curveTo(-72.5,65.1,-64.2,6.7).lineTo(50.4,22.8).curveTo(47.2,45.6,57.4,66).curveTo(67.7,86.6,87.7,97.7).lineTo(31.5,198.9).curveTo(-19.9,170.4,-46.2,117.8).closePath();
	var mask_graphics_532 = new cjs.Graphics().moveTo(-45.5,119.1).curveTo(-71.8,67.5,-64.8,10).lineTo(50.2,24.2).curveTo(47.5,46.5,57.7,66.6).curveTo(68,86.7,87.7,97.7).lineTo(31.5,198.9).curveTo(-19.1,170.8,-45.5,119.1).closePath();
	var mask_graphics_533 = new cjs.Graphics().moveTo(-44.7,120.6).curveTo(-71,70.1,-65,13.3).lineTo(50.1,25.4).curveTo(47.8,47.5,57.9,67.1).curveTo(68.2,86.9,87.7,97.7).lineTo(31.5,198.9).curveTo(-18.3,171.2,-44.7,120.6).closePath();
	var mask_graphics_534 = new cjs.Graphics().moveTo(-43.9,122.1).curveTo(-70.3,72.5,-65.4,16.6).lineTo(49.9,26.6).curveTo(48.1,48.4,58.3,67.6).curveTo(68.6,87,87.7,97.7).lineTo(31.5,198.9).curveTo(-17.5,171.7,-43.9,122.1).closePath();
	var mask_graphics_535 = new cjs.Graphics().moveTo(-43.2,123.6).curveTo(-69.5,75,-65.7,19.8).lineTo(49.8,28).curveTo(48.3,49.4,58.6,68.3).curveTo(68.9,87.3,87.7,97.7).lineTo(31.5,198.9).curveTo(-16.8,172.2,-43.2,123.6).closePath();
	var mask_graphics_536 = new cjs.Graphics().moveTo(-42.4,125.1).curveTo(-68.7,77.5,-65.8,23.2).lineTo(49.8,29.2).curveTo(48.7,50.4,58.9,68.9).curveTo(69.2,87.4,87.7,97.7).lineTo(31.5,198.9).curveTo(-16,172.6,-42.4,125.1).closePath();
	var mask_graphics_537 = new cjs.Graphics().moveTo(-41.6,126.4).curveTo(-67.9,80,-66,26.5).lineTo(49.7,30.5).curveTo(49,51.4,59.2,69.4).curveTo(69.4,87.6,87.7,97.7).lineTo(31.5,198.9).curveTo(-15.2,173,-41.6,126.4).closePath();
	var mask_graphics_538 = new cjs.Graphics().moveTo(-40.8,127.9).curveTo(-67.1,82.4,-66.1,29.9).lineTo(49.7,31.8).curveTo(49.3,52.3,59.6,69.9).curveTo(69.9,87.7,87.7,97.7).lineTo(31.5,198.9).curveTo(-14.4,173.4,-40.8,127.9).closePath();
	var mask_graphics_539 = new cjs.Graphics().moveTo(-40,129.3).curveTo(-66.1,84.8,-66.1,33.1).lineTo(49.7,33.1).curveTo(49.7,53.3,59.8,70.6).curveTo(70.1,88,87.7,97.7).lineTo(31.5,198.9).curveTo(-13.7,173.8,-40,129.3).closePath();
	var mask_graphics_540 = new cjs.Graphics().moveTo(-39,130.8).curveTo(-65.2,87.1,-66.1,36.4).lineTo(49.7,34.5).curveTo(50.1,54.2,60.2,71.2).curveTo(70.4,88.1,87.7,97.7).lineTo(31.5,198.9).curveTo(-12.9,174.4,-39,130.8).closePath();
	var mask_graphics_541 = new cjs.Graphics().moveTo(-38.2,132.1).curveTo(-64.2,89.6,-66,39.7).lineTo(49.7,35.7).curveTo(50.4,55.1,60.5,71.6).curveTo(70.7,88.2,87.7,97.7).lineTo(31.5,198.9).curveTo(-12.1,174.8,-38.2,132.1).closePath();
	var mask_graphics_542 = new cjs.Graphics().moveTo(-37.3,133.6).curveTo(-63.3,92,-65.8,43).lineTo(49.8,37).curveTo(50.8,56,60.9,72.1).curveTo(71.1,88.4,87.7,97.7).lineTo(31.5,198.9).curveTo(-11.3,175.2,-37.3,133.6).closePath();
	var mask_graphics_543 = new cjs.Graphics().moveTo(-36.5,135).curveTo(-62.3,94.3,-65.7,46.4).lineTo(49.8,38.3).curveTo(51.2,56.9,61.2,72.8).curveTo(71.3,88.6,87.7,97.7).lineTo(31.5,198.9).curveTo(-10.6,175.6,-36.5,135).closePath();
	var mask_graphics_544 = new cjs.Graphics().moveTo(-35.5,136.3).curveTo(-61.2,96.8,-65.4,49.6).lineTo(49.9,39.6).curveTo(51.6,57.9,61.6,73.3).curveTo(71.6,88.8,87.7,97.7).lineTo(31.5,198.9).curveTo(-9.8,176,-35.5,136.3).closePath();
	var mask_graphics_545 = new cjs.Graphics().moveTo(-34.7,137.7).curveTo(-60.3,99.1,-65,52.9).lineTo(50.1,40.8).curveTo(52,58.8,61.9,73.9).curveTo(71.9,88.9,87.7,97.7).lineTo(31.5,198.9).curveTo(-9,176.4,-34.7,137.7).closePath();
	var mask_graphics_546 = new cjs.Graphics().moveTo(-33.7,139.1).curveTo(-59.2,101.4,-64.8,56.3).lineTo(50.2,42.1).curveTo(52.4,59.7,62.3,74.3).curveTo(72.2,89,87.7,97.7).lineTo(31.5,198.9).curveTo(-8.3,176.9,-33.7,139.1).closePath();
	var mask_graphics_547 = new cjs.Graphics().moveTo(-32.8,140.5).curveTo(-58.1,103.7,-64.2,59.5).lineTo(50.4,43.4).curveTo(52.8,60.6,62.7,74.8).curveTo(72.6,89.2,87.7,97.7).lineTo(31.5,198.9).curveTo(-7.5,177.3,-32.8,140.5).closePath();
	var mask_graphics_548 = new cjs.Graphics().moveTo(-31.8,141.9).curveTo(-56.9,106,-63.8,62.8).lineTo(50.6,44.6).curveTo(53.2,61.6,62.9,75.5).curveTo(72.8,89.4,87.7,97.7).lineTo(31.5,198.9).curveTo(-6.7,177.7,-31.8,141.9).closePath();
	var mask_graphics_549 = new cjs.Graphics().moveTo(-30.9,143.2).curveTo(-55.8,108.3,-63.3,66).lineTo(50.8,46).curveTo(53.7,62.4,63.4,75.9).curveTo(73.1,89.6,87.7,97.7).lineTo(31.5,198.9).curveTo(-6,178.2,-30.9,143.2).closePath();
	var mask_graphics_550 = new cjs.Graphics().moveTo(-30,144.6).curveTo(-54.6,110.6,-62.6,69.3).lineTo(51,47.2).curveTo(54.1,63.3,63.8,76.4).curveTo(73.4,89.7,87.7,97.7).lineTo(31.5,198.9).curveTo(-5.2,178.6,-30,144.6).closePath();
	var mask_graphics_551 = new cjs.Graphics().moveTo(-29,145.8).curveTo(-53.4,112.7,-61.9,72.5).lineTo(51.3,48.4).curveTo(54.7,64.1,64.2,77).curveTo(73.6,89.9,87.7,97.7).lineTo(31.5,198.9).curveTo(-4.5,179,-29,145.8).closePath();
	var mask_graphics_552 = new cjs.Graphics().moveTo(-27.9,147.1).curveTo(-52.2,115,-61.2,75.8).lineTo(51.6,49.8).curveTo(55.1,64.9,64.4,77.5).curveTo(73.9,90.1,87.7,97.7).lineTo(31.5,198.9).curveTo(-3.7,179.4,-27.9,147.1).closePath();
	var mask_graphics_553 = new cjs.Graphics().moveTo(-27,148.5).curveTo(-50.9,117.2,-60.4,79).lineTo(51.8,51).curveTo(55.6,65.9,65,78.1).curveTo(74.3,90.3,87.7,97.7).lineTo(31.5,198.9).curveTo(-2.9,179.8,-27,148.5).closePath();
	var mask_graphics_554 = new cjs.Graphics().moveTo(-26,149.7).curveTo(-49.7,119.4,-59.6,82.1).lineTo(52.2,52.2).curveTo(56,66.7,65.2,78.5).curveTo(74.6,90.4,87.7,97.7).lineTo(31.5,198.9).curveTo(-2.2,180.2,-26,149.7).closePath();
	var mask_graphics_555 = new cjs.Graphics().moveTo(-58.8,85.4).lineTo(52.5,53.4).curveTo(60.9,82.8,87.7,97.7).lineTo(31.5,198.9).curveTo(-37.1,160.8,-58.8,85.4).closePath();
	var mask_graphics_556 = new cjs.Graphics().moveTo(-57.9,88.5).lineTo(52.9,54.6).curveTo(61.6,83.2,87.7,97.7).lineTo(31.5,198.9).curveTo(-35.4,161.8,-57.9,88.5).closePath();
	var mask_graphics_557 = new cjs.Graphics().moveTo(-56.8,91.8).lineTo(53.3,55.9).curveTo(62.3,83.6,87.7,97.7).lineTo(31.5,198.9).curveTo(-33.7,162.7,-56.8,91.8).closePath();
	var mask_graphics_558 = new cjs.Graphics().moveTo(-55.8,94.9).lineTo(53.7,57.1).curveTo(62.9,83.9,87.7,97.7).lineTo(31.5,198.9).curveTo(-32.1,163.7,-55.8,94.9).closePath();
	var mask_graphics_559 = new cjs.Graphics().moveTo(-54.7,98).lineTo(54.1,58.3).curveTo(63.6,84.3,87.7,97.7).lineTo(31.5,198.9).curveTo(-30.5,164.6,-54.7,98).closePath();
	var mask_graphics_560 = new cjs.Graphics().moveTo(-53.5,101.1).lineTo(54.6,59.5).curveTo(64.2,84.6,87.7,97.7).lineTo(31.5,198.9).curveTo(-28.7,165.4,-53.5,101.1).closePath();
	var mask_graphics_561 = new cjs.Graphics().moveTo(-52.3,104.1).lineTo(55.1,60.7).curveTo(64.8,85,87.7,97.7).lineTo(31.5,198.9).curveTo(-27.1,166.4,-52.3,104.1).closePath();
	var mask_graphics_562 = new cjs.Graphics().moveTo(-51.1,107.2).lineTo(55.5,62).curveTo(65.5,85.4,87.7,97.7).lineTo(31.5,198.9).curveTo(-25.5,167.3,-51.1,107.2).closePath();
	var mask_graphics_563 = new cjs.Graphics().moveTo(-49.7,110.2).lineTo(56,63.2).curveTo(66.1,85.7,87.7,97.7).lineTo(31.5,198.9).curveTo(-23.9,168.1,-49.7,110.2).closePath();
	var mask_graphics_564 = new cjs.Graphics().moveTo(-48.4,113.3).lineTo(56.6,64.3).curveTo(66.7,86.1,87.7,97.7).lineTo(31.5,198.9).curveTo(-22.4,169.1,-48.4,113.3).closePath();
	var mask_graphics_565 = new cjs.Graphics().moveTo(-46.9,116.3).lineTo(57.1,65.5).curveTo(67.3,86.3,87.7,97.7).lineTo(31.5,198.9).curveTo(-20.7,169.9,-46.9,116.3).closePath();
	var mask_graphics_566 = new cjs.Graphics().moveTo(-45.4,119.2).lineTo(57.7,66.6).curveTo(68,86.7,87.7,97.7).lineTo(31.5,198.9).curveTo(-19.1,170.8,-45.4,119.2).closePath();
	var mask_graphics_567 = new cjs.Graphics().moveTo(-43.9,122.1).lineTo(58.3,67.8).curveTo(68.6,87,87.7,97.7).lineTo(31.5,198.9).curveTo(-17.5,171.7,-43.9,122.1).closePath();
	var mask_graphics_568 = new cjs.Graphics().moveTo(-42.3,125.1).lineTo(58.9,68.9).curveTo(69.2,87.4,87.7,97.7).lineTo(31.5,198.9).curveTo(-16,172.6,-42.3,125.1).closePath();
	var mask_graphics_569 = new cjs.Graphics().moveTo(-40.7,127.9).lineTo(59.6,70.1).curveTo(69.9,87.7,87.7,97.7).lineTo(31.5,198.9).curveTo(-14.4,173.4,-40.7,127.9).closePath();
	var mask_graphics_570 = new cjs.Graphics().moveTo(-39,130.8).lineTo(60.2,71.2).curveTo(70.4,88.1,87.7,97.7).lineTo(31.5,198.9).curveTo(-12.9,174.4,-39,130.8).closePath();
	var mask_graphics_571 = new cjs.Graphics().moveTo(-37.3,133.6).lineTo(60.9,72.3).curveTo(71.1,88.4,87.7,97.7).lineTo(31.5,198.9).curveTo(-11.3,175.2,-37.3,133.6).closePath();
	var mask_graphics_572 = new cjs.Graphics().moveTo(-35.5,136.3).lineTo(61.6,73.3).curveTo(71.6,88.8,87.7,97.7).lineTo(31.5,198.9).curveTo(-9.8,176,-35.5,136.3).closePath();
	var mask_graphics_573 = new cjs.Graphics().moveTo(-33.7,139.1).lineTo(62.3,74.4).curveTo(72.2,89,87.7,97.7).lineTo(31.5,198.9).curveTo(-8.3,176.9,-33.7,139.1).closePath();
	var mask_graphics_574 = new cjs.Graphics().moveTo(-31.8,141.9).lineTo(63.1,75.5).curveTo(72.8,89.4,87.7,97.7).lineTo(31.5,198.9).curveTo(-6.7,177.7,-31.8,141.9).closePath();
	var mask_graphics_575 = new cjs.Graphics().moveTo(-30,144.6).lineTo(63.8,76.4).curveTo(73.4,89.7,87.7,97.7).lineTo(31.5,198.9).curveTo(-5.2,178.6,-30,144.6).closePath();
	var mask_graphics_576 = new cjs.Graphics().moveTo(-27.9,147.3).lineTo(64.6,77.5).curveTo(73.9,90.1,87.7,97.7).lineTo(31.5,198.9).curveTo(-3.7,179.4,-27.9,147.3).closePath();
	var mask_graphics_577 = new cjs.Graphics().moveTo(-25.9,149.8).lineTo(65.4,78.6).curveTo(74.6,90.4,87.7,97.7).lineTo(31.5,198.9).curveTo(-2.2,180.2,-25.9,149.8).closePath();
	var mask_graphics_578 = new cjs.Graphics().moveTo(-23.9,152.4).lineTo(66.1,79.6).curveTo(75.1,90.7,87.7,97.7).lineTo(31.5,198.9).curveTo(-0.7,181.1,-23.9,152.4).closePath();
	var mask_graphics_579 = new cjs.Graphics().moveTo(-21.7,155).lineTo(67,80.5).curveTo(75.7,91.1,87.7,97.7).lineTo(31.5,198.9).curveTo(0.8,181.9,-21.7,155).closePath();
	var mask_graphics_580 = new cjs.Graphics().moveTo(-19.7,157.4).lineTo(67.8,81.6).curveTo(76.4,91.3,87.7,97.7).lineTo(31.5,198.9).curveTo(2.3,182.8,-19.7,157.4).closePath();
	var mask_graphics_581 = new cjs.Graphics().moveTo(-17.4,160).lineTo(68.6,82.5).curveTo(76.9,91.6,87.7,97.7).lineTo(31.5,198.9).curveTo(3.8,183.6,-17.4,160).closePath();
	var mask_graphics_582 = new cjs.Graphics().moveTo(-15.2,162.4).lineTo(69.4,83.5).curveTo(77.4,92,87.7,97.7).lineTo(31.5,198.9).curveTo(5.3,184.4,-15.2,162.4).closePath();
	var mask_graphics_583 = new cjs.Graphics().moveTo(-12.9,164.9).lineTo(70.4,84.4).curveTo(78.1,92.3,87.7,97.7).lineTo(31.5,198.9).curveTo(6.7,185.2,-12.9,164.9).closePath();
	var mask_graphics_584 = new cjs.Graphics().moveTo(-10.6,167.2).lineTo(71.3,85.3).curveTo(78.7,92.7,87.7,97.7).lineTo(31.5,198.9).curveTo(8.2,186,-10.6,167.2).closePath();
	var mask_graphics_585 = new cjs.Graphics().moveTo(-8.3,169.5).lineTo(72.2,86.2).curveTo(79.2,93,87.7,97.7).lineTo(31.5,198.9).curveTo(9.7,186.8,-8.3,169.5).closePath();
	var mask_graphics_586 = new cjs.Graphics().moveTo(-5.8,171.8).lineTo(73.1,87.1).curveTo(79.7,93.2,87.7,97.7).lineTo(31.5,198.9).curveTo(11.2,187.6,-5.8,171.8).closePath();
	var mask_graphics_587 = new cjs.Graphics().moveTo(-3.4,174).lineTo(74.1,88).curveTo(80.3,93.6,87.7,97.7).lineTo(31.5,198.9).curveTo(12.7,188.4,-3.4,174).closePath();
	var mask_graphics_588 = new cjs.Graphics().moveTo(-0.8,176.3).lineTo(75,88.8).curveTo(81,93.9,87.7,97.7).lineTo(31.5,198.9).curveTo(14.2,189.3,-0.8,176.3).closePath();
	var mask_graphics_589 = new cjs.Graphics().moveTo(1.6,178.3).lineTo(76.1,89.6).curveTo(81.5,94.2,87.7,97.7).lineTo(31.5,198.9).curveTo(15.5,190.1,1.6,178.3).closePath();
	var mask_graphics_590 = new cjs.Graphics().moveTo(4.2,180.5).lineTo(77,90.5).curveTo(82,94.6,87.7,97.7).lineTo(31.5,198.9).curveTo(17,190.9,4.2,180.5).closePath();
	var mask_graphics_591 = new cjs.Graphics().moveTo(6.7,182.5).lineTo(78,91.2).curveTo(82.6,94.9,87.7,97.7).lineTo(31.5,198.9).curveTo(18.5,191.7,6.7,182.5).closePath();
	var mask_graphics_592 = new cjs.Graphics().moveTo(9.3,184.5).lineTo(79.1,92).curveTo(83.1,95.1,87.7,97.7).lineTo(31.5,198.9).curveTo(20,192.5,9.3,184.5).closePath();
	var mask_graphics_593 = new cjs.Graphics().moveTo(12,186.5).lineTo(80.1,92.8).curveTo(83.8,95.5,87.7,97.7).lineTo(31.5,198.9).curveTo(21.4,193.3,12,186.5).closePath();
	var mask_graphics_594 = new cjs.Graphics().moveTo(14.7,188.4).lineTo(81.1,93.5).curveTo(84.3,95.8,87.7,97.7).lineTo(31.5,198.9).curveTo(22.9,194.1,14.7,188.4).closePath();
	var mask_graphics_595 = new cjs.Graphics().moveTo(17.4,190.3).lineTo(82.2,94.3).curveTo(84.9,96.1,87.7,97.7).lineTo(31.5,198.9).curveTo(24.4,194.9,17.4,190.3).closePath();
	var mask_graphics_596 = new cjs.Graphics().moveTo(20.3,192.1).lineTo(83.3,95).lineTo(87.7,97.7).lineTo(31.5,198.9).curveTo(25.7,195.8,20.3,192.1).closePath();
	var mask_graphics_597 = new cjs.Graphics().moveTo(23,193.9).lineTo(84.3,95.7).curveTo(86,96.8,87.7,97.7).lineTo(31.5,198.9).curveTo(27.2,196.6,23,193.9).closePath();
	var mask_graphics_598 = new cjs.Graphics().moveTo(25.8,195.6).lineTo(85.4,96.4).lineTo(87.7,97.7).lineTo(31.5,198.9).curveTo(28.7,197.4,25.8,195.6).closePath();

	this.timeline.addTween(cjs.Tween.get(mask).to({graphics:mask_graphics_0,x:88.3976,y:198.4722}).wait(1).to({graphics:mask_graphics_1,x:87.7476,y:198.8722}).wait(1).to({graphics:mask_graphics_2,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_3,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_4,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_5,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_6,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_7,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_8,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_9,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_10,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_11,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_12,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_13,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_14,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_15,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_16,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_17,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_18,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_19,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_20,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_21,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_22,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_23,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_24,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_25,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_26,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_27,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_28,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_29,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_30,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_31,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_32,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_33,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_34,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_35,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_36,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_37,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_38,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_39,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_40,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_41,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_42,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_43,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_44,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_45,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_46,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_47,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_48,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_49,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_50,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_51,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_52,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_53,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_54,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_55,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_56,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_57,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_58,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_59,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_60,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_61,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_62,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_63,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_64,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_65,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_66,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_67,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_68,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_69,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_70,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_71,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_72,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_73,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_74,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_75,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_76,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_77,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_78,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_79,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_80,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_81,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_82,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_83,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_84,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_85,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_86,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_87,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_88,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_89,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_90,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_91,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_92,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_93,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_94,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_95,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_96,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_97,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_98,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_99,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_100,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_101,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_102,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_103,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_104,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_105,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_106,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_107,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_108,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_109,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_110,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_111,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_112,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_113,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_114,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_115,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_116,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_117,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_118,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_119,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_120,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_121,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_122,x:88.2714,y:198.8722}).wait(1).to({graphics:mask_graphics_123,x:88.8808,y:198.8722}).wait(1).to({graphics:mask_graphics_124,x:89.4225,y:198.8722}).wait(1).to({graphics:mask_graphics_125,x:90.0319,y:198.8722}).wait(1).to({graphics:mask_graphics_126,x:90.5736,y:198.8722}).wait(1).to({graphics:mask_graphics_127,x:91.183,y:198.8722}).wait(1).to({graphics:mask_graphics_128,x:91.7924,y:198.8722}).wait(1).to({graphics:mask_graphics_129,x:92.4018,y:198.8722}).wait(1).to({graphics:mask_graphics_130,x:93.0112,y:198.8722}).wait(1).to({graphics:mask_graphics_131,x:93.6207,y:198.8722}).wait(1).to({graphics:mask_graphics_132,x:94.2301,y:198.8722}).wait(1).to({graphics:mask_graphics_133,x:94.8395,y:198.8722}).wait(1).to({graphics:mask_graphics_134,x:95.4489,y:198.8722}).wait(1).to({graphics:mask_graphics_135,x:96.0583,y:198.8722}).wait(1).to({graphics:mask_graphics_136,x:96.6677,y:198.8722}).wait(1).to({graphics:mask_graphics_137,x:97.2771,y:198.8722}).wait(1).to({graphics:mask_graphics_138,x:97.9542,y:198.8722}).wait(1).to({graphics:mask_graphics_139,x:98.5636,y:198.8722}).wait(1).to({graphics:mask_graphics_140,x:99.173,y:198.8722}).wait(1).to({graphics:mask_graphics_141,x:99.8502,y:198.8722}).wait(1).to({graphics:mask_graphics_142,x:100.4596,y:198.8722}).wait(1).to({graphics:mask_graphics_143,x:101.1367,y:198.8722}).wait(1).to({graphics:mask_graphics_144,x:101.7461,y:198.8722}).wait(1).to({graphics:mask_graphics_145,x:102.3555,y:198.8722}).wait(1).to({graphics:mask_graphics_146,x:103.0326,y:198.8722}).wait(1).to({graphics:mask_graphics_147,x:103.642,y:198.8722}).wait(1).to({graphics:mask_graphics_148,x:104.3192,y:198.8722}).wait(1).to({graphics:mask_graphics_149,x:104.9286,y:198.8722}).wait(1).to({graphics:mask_graphics_150,x:105.6057,y:198.8722}).wait(1).to({graphics:mask_graphics_151,x:107.2308,y:198.8722}).wait(1).to({graphics:mask_graphics_152,x:108.9236,y:198.8722}).wait(1).to({graphics:mask_graphics_153,x:110.5487,y:198.8722}).wait(1).to({graphics:mask_graphics_154,x:112.2415,y:198.8722}).wait(1).to({graphics:mask_graphics_155,x:113.8665,y:198.8722}).wait(1).to({graphics:mask_graphics_156,x:115.4916,y:198.8722}).wait(1).to({graphics:mask_graphics_157,x:117.1844,y:198.8722}).wait(1).to({graphics:mask_graphics_158,x:118.8095,y:198.8722}).wait(1).to({graphics:mask_graphics_159,x:120.4346,y:198.8722}).wait(1).to({graphics:mask_graphics_160,x:122.0597,y:198.8722}).wait(1).to({graphics:mask_graphics_161,x:123.6848,y:198.8722}).wait(1).to({graphics:mask_graphics_162,x:125.3099,y:198.8722}).wait(1).to({graphics:mask_graphics_163,x:126.935,y:198.8722}).wait(1).to({graphics:mask_graphics_164,x:128.5601,y:198.8722}).wait(1).to({graphics:mask_graphics_165,x:130.1174,y:198.8722}).wait(1).to({graphics:mask_graphics_166,x:131.7425,y:198.8722}).wait(1).to({graphics:mask_graphics_167,x:133.2999,y:198.8722}).wait(1).to({graphics:mask_graphics_168,x:134.925,y:198.8722}).wait(1).to({graphics:mask_graphics_169,x:136.4824,y:198.8722}).wait(1).to({graphics:mask_graphics_170,x:138.0397,y:198.8722}).wait(1).to({graphics:mask_graphics_171,x:139.5971,y:198.8722}).wait(1).to({graphics:mask_graphics_172,x:141.0868,y:198.8722}).wait(1).to({graphics:mask_graphics_173,x:142.6442,y:198.8722}).wait(1).to({graphics:mask_graphics_174,x:144.1338,y:198.8722}).wait(1).to({graphics:mask_graphics_175,x:145.6912,y:198.8722}).wait(1).to({graphics:mask_graphics_176,x:147.1809,y:198.8722}).wait(1).to({graphics:mask_graphics_177,x:148.6705,y:198.8722}).wait(1).to({graphics:mask_graphics_178,x:150.0925,y:198.8722}).wait(1).to({graphics:mask_graphics_179,x:151.5821,y:198.8722}).wait(1).to({graphics:mask_graphics_180,x:153.0041,y:198.8722}).wait(1).to({graphics:mask_graphics_181,x:154.426,y:198.8722}).wait(1).to({graphics:mask_graphics_182,x:155.848,y:198.8722}).wait(1).to({graphics:mask_graphics_183,x:157.2022,y:198.8722}).wait(1).to({graphics:mask_graphics_184,x:158.6242,y:198.8722}).wait(1).to({graphics:mask_graphics_185,x:159.9784,y:198.8722}).wait(1).to({graphics:mask_graphics_186,x:161.3327,y:198.8722}).wait(1).to({graphics:mask_graphics_187,x:162.6869,y:198.8722}).wait(1).to({graphics:mask_graphics_188,x:163.9734,y:198.8722}).wait(1).to({graphics:mask_graphics_189,x:165.26,y:198.8722}).wait(1).to({graphics:mask_graphics_190,x:166.5465,y:198.8722}).wait(1).to({graphics:mask_graphics_191,x:167.7653,y:198.8722}).wait(1).to({graphics:mask_graphics_192,x:169.0518,y:198.8722}).wait(1).to({graphics:mask_graphics_193,x:170.2707,y:198.8722}).wait(1).to({graphics:mask_graphics_194,x:171.4895,y:198.8722}).wait(1).to({graphics:mask_graphics_195,x:172.6406,y:198.8722}).wait(1).to({graphics:mask_graphics_196,x:173.7917,y:198.8722}).wait(1).to({graphics:mask_graphics_197,x:174.9428,y:198.8722}).wait(1).to({graphics:mask_graphics_198,x:176.0262,y:198.8722}).wait(1).to({graphics:mask_graphics_199,x:177.1773,y:198.8722}).wait(1).to({graphics:mask_graphics_200,x:178.193,y:198.8722}).wait(1).to({graphics:mask_graphics_201,x:179.2764,y:198.8722}).wait(1).to({graphics:mask_graphics_202,x:180.292,y:198.8722}).wait(1).to({graphics:mask_graphics_203,x:181.3077,y:198.8722}).wait(1).to({graphics:mask_graphics_204,x:182.3234,y:198.8722}).wait(1).to({graphics:mask_graphics_205,x:183.2714,y:198.8722}).wait(1).to({graphics:mask_graphics_206,x:184.2193,y:198.8722}).wait(1).to({graphics:mask_graphics_207,x:185.0996,y:198.8722}).wait(1).to({graphics:mask_graphics_208,x:185.9798,y:198.8722}).wait(1).to({graphics:mask_graphics_209,x:186.8601,y:198.8722}).wait(1).to({graphics:mask_graphics_210,x:187.6726,y:198.8722}).wait(1).to({graphics:mask_graphics_211,x:188.4852,y:198.8722}).wait(1).to({graphics:mask_graphics_212,x:189.2977,y:198.8722}).wait(1).to({graphics:mask_graphics_213,x:190.0426,y:198.8722}).wait(1).to({graphics:mask_graphics_214,x:190.7874,y:198.8722}).wait(1).to({graphics:mask_graphics_215,x:191.5322,y:198.8722}).wait(1).to({graphics:mask_graphics_216,x:192.2094,y:198.8722}).wait(1).to({graphics:mask_graphics_217,x:192.8865,y:198.8722}).wait(1).to({graphics:mask_graphics_218,x:194.1053,y:198.8722}).wait(1).to({graphics:mask_graphics_219,x:194.7147,y:198.8722}).wait(1).to({graphics:mask_graphics_220,x:195.2564,y:198.8722}).wait(1).to({graphics:mask_graphics_221,x:195.7304,y:198.8722}).wait(1).to({graphics:mask_graphics_222,x:196.2721,y:198.8722}).wait(1).to({graphics:mask_graphics_223,x:196.7461,y:198.8722}).wait(1).to({graphics:mask_graphics_224,x:197.1523,y:198.8722}).wait(1).to({graphics:mask_graphics_225,x:197.5586,y:198.8722}).wait(1).to({graphics:mask_graphics_226,x:197.9649,y:198.8722}).wait(1).to({graphics:mask_graphics_227,x:198.3034,y:198.8722}).wait(1).to({graphics:mask_graphics_228,x:198.642,y:198.8722}).wait(1).to({graphics:mask_graphics_229,x:198.9806,y:198.8722}).wait(1).to({graphics:mask_graphics_230,x:199.2514,y:198.8722}).wait(1).to({graphics:mask_graphics_231,x:199.4545,y:198.8722}).wait(1).to({graphics:mask_graphics_232,x:199.7254,y:198.8722}).wait(1).to({graphics:mask_graphics_233,x:199.8608,y:198.8722}).wait(1).to({graphics:mask_graphics_234,x:200.0639,y:198.8722}).wait(1).to({graphics:mask_graphics_235,x:200.1994,y:198.8722}).wait(1).to({graphics:mask_graphics_236,x:200.2671,y:198.8722}).wait(1).to({graphics:mask_graphics_237,x:200.3348,y:198.8722}).wait(1).to({graphics:mask_graphics_238,x:200.4025,y:198.8722}).wait(1).to({graphics:mask_graphics_239,x:200.4025,y:198.8722}).wait(1).to({graphics:mask_graphics_240,x:200.3469,y:198.8424}).wait(1).to({graphics:mask_graphics_241,x:200.3633,y:198.8424}).wait(1).to({graphics:mask_graphics_242,x:200.3548,y:198.8424}).wait(1).to({graphics:mask_graphics_243,x:200.3875,y:198.8424}).wait(1).to({graphics:mask_graphics_244,x:200.396,y:198.8424}).wait(1).to({graphics:mask_graphics_245,x:200.4059,y:198.8424}).wait(1).to({graphics:mask_graphics_246,x:200.3917,y:198.8424}).wait(1).to({graphics:mask_graphics_247,x:200.4579,y:198.8424}).wait(1).to({graphics:mask_graphics_248,x:200.4453,y:198.8424}).wait(1).to({graphics:mask_graphics_249,x:200.4541,y:198.8424}).wait(1).to({graphics:mask_graphics_250,x:200.4865,y:198.8424}).wait(1).to({graphics:mask_graphics_251,x:200.4747,y:198.8424}).wait(1).to({graphics:mask_graphics_252,x:200.4914,y:198.8424}).wait(1).to({graphics:mask_graphics_253,x:200.5222,y:198.8424}).wait(1).to({graphics:mask_graphics_254,x:200.5076,y:198.8424}).wait(1).to({graphics:mask_graphics_255,x:200.5083,y:198.8424}).wait(1).to({graphics:mask_graphics_256,x:200.5969,y:198.8424}).wait(1).to({graphics:mask_graphics_257,x:200.5434,y:198.8424}).wait(1).to({graphics:mask_graphics_258,x:200.5359,y:198.8424}).wait(1).to({graphics:mask_graphics_259,x:200.6166,y:198.8424}).wait(1).to({graphics:mask_graphics_260,x:200.5823,y:198.8424}).wait(1).to({graphics:mask_graphics_261,x:200.563,y:198.8424}).wait(1).to({graphics:mask_graphics_262,x:200.6298,y:198.8424}).wait(1).to({graphics:mask_graphics_263,x:200.5864,y:198.5924}).wait(1).to({graphics:mask_graphics_264,x:200.5649,y:198.5924}).wait(1).to({graphics:mask_graphics_265,x:200.6251,y:198.5924}).wait(1).to({graphics:mask_graphics_266,x:200.5871,y:198.5924}).wait(1).to({graphics:mask_graphics_267,x:200.5385,y:198.5924}).wait(1).to({graphics:mask_graphics_268,x:200.6189,y:198.5924}).wait(1).to({graphics:mask_graphics_269,x:200.3616,y:198.5924}).wait(1).to({graphics:mask_graphics_270,x:200.3061,y:198.5924}).wait(1).to({graphics:mask_graphics_271,x:200.4414,y:198.5924}).wait(1).to({graphics:mask_graphics_272,x:200.4047,y:198.5924}).wait(1).to({graphics:mask_graphics_273,x:200.3493,y:198.5924}).wait(1).to({graphics:mask_graphics_274,x:200.295,y:198.5924}).wait(1).to({graphics:mask_graphics_275,x:200.3525,y:198.5924}).wait(1).to({graphics:mask_graphics_276,x:200.2949,y:198.5924}).wait(1).to({graphics:mask_graphics_277,x:200.2837,y:198.5924}).wait(1).to({graphics:mask_graphics_278,x:200.398,y:198.5924}).wait(1).to({graphics:mask_graphics_279,x:200.4116,y:198.5924}).wait(1).to({graphics:mask_graphics_280,x:200.3273,y:198.5924}).wait(1).to({graphics:mask_graphics_281,x:200.2384,y:198.5924}).wait(1).to({graphics:mask_graphics_282,x:200.4084,y:198.5924}).wait(1).to({graphics:mask_graphics_283,x:200.3038,y:198.5924}).wait(1).to({graphics:mask_graphics_284,x:200.2566,y:198.5924}).wait(1).to({graphics:mask_graphics_285,x:200.4209,y:198.5924}).wait(1).to({graphics:mask_graphics_286,x:200.3943,y:198.5924}).wait(1).to({graphics:mask_graphics_287,x:200.382,y:198.5924}).wait(1).to({graphics:mask_graphics_288,x:200.3154,y:198.5924}).wait(1).to({graphics:mask_graphics_289,x:200.4811,y:198.5924}).wait(1).to({graphics:mask_graphics_290,x:200.4734,y:198.5924}).wait(1).to({graphics:mask_graphics_291,x:200.4167,y:198.5924}).wait(1).to({graphics:mask_graphics_292,x:200.5181,y:198.5924}).wait(1).to({graphics:mask_graphics_293,x:200.5424,y:198.5924}).wait(1).to({graphics:mask_graphics_294,x:200.506,y:198.5924}).wait(1).to({graphics:mask_graphics_295,x:200.4493,y:198.5924}).wait(1).to({graphics:mask_graphics_296,x:200.5666,y:198.5924}).wait(1).to({graphics:mask_graphics_297,x:200.519,y:198.5924}).wait(1).to({graphics:mask_graphics_298,x:200.5034,y:198.5924}).wait(1).to({graphics:mask_graphics_299,x:200.63,y:198.5924}).wait(1).to({graphics:mask_graphics_300,x:200.63,y:198.5924}).wait(1).to({graphics:mask_graphics_301,x:200.5034,y:198.5924}).wait(1).to({graphics:mask_graphics_302,x:200.519,y:198.5924}).wait(1).to({graphics:mask_graphics_303,x:200.5666,y:198.5924}).wait(1).to({graphics:mask_graphics_304,x:200.4493,y:198.5924}).wait(1).to({graphics:mask_graphics_305,x:200.506,y:198.5924}).wait(1).to({graphics:mask_graphics_306,x:200.5424,y:198.5924}).wait(1).to({graphics:mask_graphics_307,x:200.5181,y:198.5924}).wait(1).to({graphics:mask_graphics_308,x:200.4167,y:198.5924}).wait(1).to({graphics:mask_graphics_309,x:200.4734,y:198.5924}).wait(1).to({graphics:mask_graphics_310,x:200.4811,y:198.5924}).wait(1).to({graphics:mask_graphics_311,x:200.3154,y:198.5924}).wait(1).to({graphics:mask_graphics_312,x:200.382,y:198.5924}).wait(1).to({graphics:mask_graphics_313,x:200.3943,y:198.5924}).wait(1).to({graphics:mask_graphics_314,x:200.4209,y:198.5924}).wait(1).to({graphics:mask_graphics_315,x:200.2566,y:198.5924}).wait(1).to({graphics:mask_graphics_316,x:200.3038,y:198.5924}).wait(1).to({graphics:mask_graphics_317,x:200.4084,y:198.5924}).wait(1).to({graphics:mask_graphics_318,x:200.2384,y:198.5924}).wait(1).to({graphics:mask_graphics_319,x:200.3273,y:198.5924}).wait(1).to({graphics:mask_graphics_320,x:200.4116,y:198.5924}).wait(1).to({graphics:mask_graphics_321,x:200.398,y:198.5924}).wait(1).to({graphics:mask_graphics_322,x:200.2837,y:198.5924}).wait(1).to({graphics:mask_graphics_323,x:200.2949,y:198.5924}).wait(1).to({graphics:mask_graphics_324,x:200.3525,y:198.5924}).wait(1).to({graphics:mask_graphics_325,x:200.295,y:198.5924}).wait(1).to({graphics:mask_graphics_326,x:200.3493,y:198.5924}).wait(1).to({graphics:mask_graphics_327,x:200.4047,y:198.5924}).wait(1).to({graphics:mask_graphics_328,x:200.4414,y:198.5924}).wait(1).to({graphics:mask_graphics_329,x:200.3061,y:198.5924}).wait(1).to({graphics:mask_graphics_330,x:200.3616,y:198.5924}).wait(1).to({graphics:mask_graphics_331,x:200.6189,y:198.5924}).wait(1).to({graphics:mask_graphics_332,x:200.5385,y:198.5924}).wait(1).to({graphics:mask_graphics_333,x:200.5871,y:198.5924}).wait(1).to({graphics:mask_graphics_334,x:200.6251,y:198.5924}).wait(1).to({graphics:mask_graphics_335,x:200.5649,y:198.5924}).wait(1).to({graphics:mask_graphics_336,x:200.5864,y:198.5924}).wait(1).to({graphics:mask_graphics_337,x:200.6298,y:198.8424}).wait(1).to({graphics:mask_graphics_338,x:200.563,y:198.8424}).wait(1).to({graphics:mask_graphics_339,x:200.5823,y:198.8424}).wait(1).to({graphics:mask_graphics_340,x:200.6166,y:198.8424}).wait(1).to({graphics:mask_graphics_341,x:200.5359,y:198.8424}).wait(1).to({graphics:mask_graphics_342,x:200.5434,y:198.8424}).wait(1).to({graphics:mask_graphics_343,x:200.5969,y:198.8424}).wait(1).to({graphics:mask_graphics_344,x:200.5083,y:198.8424}).wait(1).to({graphics:mask_graphics_345,x:200.5076,y:198.8424}).wait(1).to({graphics:mask_graphics_346,x:200.5222,y:198.8424}).wait(1).to({graphics:mask_graphics_347,x:200.4914,y:198.8424}).wait(1).to({graphics:mask_graphics_348,x:200.4747,y:198.8424}).wait(1).to({graphics:mask_graphics_349,x:200.4865,y:198.8424}).wait(1).to({graphics:mask_graphics_350,x:200.4541,y:198.8424}).wait(1).to({graphics:mask_graphics_351,x:200.4453,y:198.8424}).wait(1).to({graphics:mask_graphics_352,x:200.4579,y:198.8424}).wait(1).to({graphics:mask_graphics_353,x:200.3917,y:198.8424}).wait(1).to({graphics:mask_graphics_354,x:200.4059,y:198.8424}).wait(1).to({graphics:mask_graphics_355,x:200.396,y:198.8424}).wait(1).to({graphics:mask_graphics_356,x:200.3875,y:198.8424}).wait(1).to({graphics:mask_graphics_357,x:200.3548,y:198.8424}).wait(1).to({graphics:mask_graphics_358,x:200.3633,y:198.8424}).wait(1).to({graphics:mask_graphics_359,x:200.3469,y:198.8424}).wait(1).to({graphics:mask_graphics_360,x:200.4025,y:198.8722}).wait(1).to({graphics:mask_graphics_361,x:200.4025,y:198.8722}).wait(1).to({graphics:mask_graphics_362,x:200.3348,y:198.8722}).wait(1).to({graphics:mask_graphics_363,x:200.2671,y:198.8722}).wait(1).to({graphics:mask_graphics_364,x:200.1994,y:198.8722}).wait(1).to({graphics:mask_graphics_365,x:200.0639,y:198.8722}).wait(1).to({graphics:mask_graphics_366,x:199.8608,y:198.8722}).wait(1).to({graphics:mask_graphics_367,x:199.7254,y:198.8722}).wait(1).to({graphics:mask_graphics_368,x:199.4545,y:198.8722}).wait(1).to({graphics:mask_graphics_369,x:199.2514,y:198.8722}).wait(1).to({graphics:mask_graphics_370,x:198.9806,y:198.8722}).wait(1).to({graphics:mask_graphics_371,x:198.642,y:198.8722}).wait(1).to({graphics:mask_graphics_372,x:198.3034,y:198.8722}).wait(1).to({graphics:mask_graphics_373,x:197.9649,y:198.8722}).wait(1).to({graphics:mask_graphics_374,x:197.5586,y:198.8722}).wait(1).to({graphics:mask_graphics_375,x:197.1523,y:198.8722}).wait(1).to({graphics:mask_graphics_376,x:196.7461,y:198.8722}).wait(1).to({graphics:mask_graphics_377,x:196.2721,y:198.8722}).wait(1).to({graphics:mask_graphics_378,x:195.7304,y:198.8722}).wait(1).to({graphics:mask_graphics_379,x:195.2564,y:198.8722}).wait(1).to({graphics:mask_graphics_380,x:194.7147,y:198.8722}).wait(1).to({graphics:mask_graphics_381,x:194.1053,y:198.8722}).wait(1).to({graphics:mask_graphics_382,x:192.8865,y:198.8722}).wait(1).to({graphics:mask_graphics_383,x:192.2094,y:198.8722}).wait(1).to({graphics:mask_graphics_384,x:191.5322,y:198.8722}).wait(1).to({graphics:mask_graphics_385,x:190.7874,y:198.8722}).wait(1).to({graphics:mask_graphics_386,x:190.0426,y:198.8722}).wait(1).to({graphics:mask_graphics_387,x:189.2977,y:198.8722}).wait(1).to({graphics:mask_graphics_388,x:188.4852,y:198.8722}).wait(1).to({graphics:mask_graphics_389,x:187.6726,y:198.8722}).wait(1).to({graphics:mask_graphics_390,x:186.8601,y:198.8722}).wait(1).to({graphics:mask_graphics_391,x:185.9798,y:198.8722}).wait(1).to({graphics:mask_graphics_392,x:185.0996,y:198.8722}).wait(1).to({graphics:mask_graphics_393,x:184.2193,y:198.8722}).wait(1).to({graphics:mask_graphics_394,x:183.2714,y:198.8722}).wait(1).to({graphics:mask_graphics_395,x:182.3234,y:198.8722}).wait(1).to({graphics:mask_graphics_396,x:181.3077,y:198.8722}).wait(1).to({graphics:mask_graphics_397,x:180.292,y:198.8722}).wait(1).to({graphics:mask_graphics_398,x:179.2764,y:198.8722}).wait(1).to({graphics:mask_graphics_399,x:178.193,y:198.8722}).wait(1).to({graphics:mask_graphics_400,x:177.1773,y:198.8722}).wait(1).to({graphics:mask_graphics_401,x:176.0262,y:198.8722}).wait(1).to({graphics:mask_graphics_402,x:174.9428,y:198.8722}).wait(1).to({graphics:mask_graphics_403,x:173.7917,y:198.8722}).wait(1).to({graphics:mask_graphics_404,x:172.6406,y:198.8722}).wait(1).to({graphics:mask_graphics_405,x:171.4895,y:198.8722}).wait(1).to({graphics:mask_graphics_406,x:170.2707,y:198.8722}).wait(1).to({graphics:mask_graphics_407,x:169.0518,y:198.8722}).wait(1).to({graphics:mask_graphics_408,x:167.7653,y:198.8722}).wait(1).to({graphics:mask_graphics_409,x:166.5465,y:198.8722}).wait(1).to({graphics:mask_graphics_410,x:165.26,y:198.8722}).wait(1).to({graphics:mask_graphics_411,x:163.9734,y:198.8722}).wait(1).to({graphics:mask_graphics_412,x:162.6869,y:198.8722}).wait(1).to({graphics:mask_graphics_413,x:161.3327,y:198.8722}).wait(1).to({graphics:mask_graphics_414,x:159.9784,y:198.8722}).wait(1).to({graphics:mask_graphics_415,x:158.6242,y:198.8722}).wait(1).to({graphics:mask_graphics_416,x:157.2022,y:198.8722}).wait(1).to({graphics:mask_graphics_417,x:155.848,y:198.8722}).wait(1).to({graphics:mask_graphics_418,x:154.426,y:198.8722}).wait(1).to({graphics:mask_graphics_419,x:153.0041,y:198.8722}).wait(1).to({graphics:mask_graphics_420,x:151.5821,y:198.8722}).wait(1).to({graphics:mask_graphics_421,x:150.0925,y:198.8722}).wait(1).to({graphics:mask_graphics_422,x:148.6705,y:198.8722}).wait(1).to({graphics:mask_graphics_423,x:147.1809,y:198.8722}).wait(1).to({graphics:mask_graphics_424,x:145.6912,y:198.8722}).wait(1).to({graphics:mask_graphics_425,x:144.1338,y:198.8722}).wait(1).to({graphics:mask_graphics_426,x:142.6442,y:198.8722}).wait(1).to({graphics:mask_graphics_427,x:141.0868,y:198.8722}).wait(1).to({graphics:mask_graphics_428,x:139.5971,y:198.8722}).wait(1).to({graphics:mask_graphics_429,x:138.0397,y:198.8722}).wait(1).to({graphics:mask_graphics_430,x:136.4824,y:198.8722}).wait(1).to({graphics:mask_graphics_431,x:134.925,y:198.8722}).wait(1).to({graphics:mask_graphics_432,x:133.2999,y:198.8722}).wait(1).to({graphics:mask_graphics_433,x:131.7425,y:198.8722}).wait(1).to({graphics:mask_graphics_434,x:130.1174,y:198.8722}).wait(1).to({graphics:mask_graphics_435,x:128.5601,y:198.8722}).wait(1).to({graphics:mask_graphics_436,x:126.935,y:198.8722}).wait(1).to({graphics:mask_graphics_437,x:125.3099,y:198.8722}).wait(1).to({graphics:mask_graphics_438,x:123.6848,y:198.8722}).wait(1).to({graphics:mask_graphics_439,x:122.0597,y:198.8722}).wait(1).to({graphics:mask_graphics_440,x:120.4346,y:198.8722}).wait(1).to({graphics:mask_graphics_441,x:118.8095,y:198.8722}).wait(1).to({graphics:mask_graphics_442,x:117.1844,y:198.8722}).wait(1).to({graphics:mask_graphics_443,x:115.4916,y:198.8722}).wait(1).to({graphics:mask_graphics_444,x:113.8665,y:198.8722}).wait(1).to({graphics:mask_graphics_445,x:112.2415,y:198.8722}).wait(1).to({graphics:mask_graphics_446,x:110.5487,y:198.8722}).wait(1).to({graphics:mask_graphics_447,x:108.9236,y:198.8722}).wait(1).to({graphics:mask_graphics_448,x:107.2308,y:198.8722}).wait(1).to({graphics:mask_graphics_449,x:105.6057,y:198.8722}).wait(1).to({graphics:mask_graphics_450,x:104.9286,y:198.8722}).wait(1).to({graphics:mask_graphics_451,x:104.3192,y:198.8722}).wait(1).to({graphics:mask_graphics_452,x:103.642,y:198.8722}).wait(1).to({graphics:mask_graphics_453,x:103.0326,y:198.8722}).wait(1).to({graphics:mask_graphics_454,x:102.3555,y:198.8722}).wait(1).to({graphics:mask_graphics_455,x:101.7461,y:198.8722}).wait(1).to({graphics:mask_graphics_456,x:101.1367,y:198.8722}).wait(1).to({graphics:mask_graphics_457,x:100.4596,y:198.8722}).wait(1).to({graphics:mask_graphics_458,x:99.8502,y:198.8722}).wait(1).to({graphics:mask_graphics_459,x:99.173,y:198.8722}).wait(1).to({graphics:mask_graphics_460,x:98.5636,y:198.8722}).wait(1).to({graphics:mask_graphics_461,x:97.9542,y:198.8722}).wait(1).to({graphics:mask_graphics_462,x:97.2771,y:198.8722}).wait(1).to({graphics:mask_graphics_463,x:96.6677,y:198.8722}).wait(1).to({graphics:mask_graphics_464,x:96.0583,y:198.8722}).wait(1).to({graphics:mask_graphics_465,x:95.4489,y:198.8722}).wait(1).to({graphics:mask_graphics_466,x:94.8395,y:198.8722}).wait(1).to({graphics:mask_graphics_467,x:94.2301,y:198.8722}).wait(1).to({graphics:mask_graphics_468,x:93.6207,y:198.8722}).wait(1).to({graphics:mask_graphics_469,x:93.0112,y:198.8722}).wait(1).to({graphics:mask_graphics_470,x:92.4018,y:198.8722}).wait(1).to({graphics:mask_graphics_471,x:91.7924,y:198.8722}).wait(1).to({graphics:mask_graphics_472,x:91.183,y:198.8722}).wait(1).to({graphics:mask_graphics_473,x:90.5736,y:198.8722}).wait(1).to({graphics:mask_graphics_474,x:90.0319,y:198.8722}).wait(1).to({graphics:mask_graphics_475,x:89.4225,y:198.8722}).wait(1).to({graphics:mask_graphics_476,x:88.8808,y:198.8722}).wait(1).to({graphics:mask_graphics_477,x:88.2714,y:198.8722}).wait(1).to({graphics:mask_graphics_478,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_479,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_480,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_481,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_482,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_483,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_484,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_485,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_486,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_487,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_488,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_489,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_490,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_491,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_492,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_493,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_494,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_495,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_496,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_497,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_498,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_499,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_500,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_501,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_502,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_503,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_504,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_505,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_506,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_507,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_508,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_509,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_510,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_511,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_512,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_513,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_514,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_515,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_516,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_517,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_518,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_519,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_520,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_521,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_522,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_523,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_524,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_525,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_526,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_527,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_528,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_529,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_530,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_531,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_532,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_533,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_534,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_535,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_536,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_537,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_538,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_539,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_540,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_541,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_542,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_543,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_544,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_545,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_546,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_547,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_548,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_549,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_550,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_551,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_552,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_553,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_554,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_555,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_556,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_557,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_558,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_559,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_560,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_561,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_562,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_563,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_564,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_565,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_566,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_567,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_568,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_569,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_570,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_571,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_572,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_573,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_574,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_575,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_576,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_577,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_578,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_579,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_580,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_581,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_582,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_583,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_584,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_585,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_586,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_587,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_588,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_589,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_590,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_591,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_592,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_593,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_594,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_595,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_596,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_597,x:87.7297,y:198.8722}).wait(1).to({graphics:mask_graphics_598,x:87.7476,y:198.8722}).wait(1).to({graphics:null,x:0,y:0}).wait(1));

	// borderline
	this.instance = new lib.CachedBmp_9();
	this.instance.setTransform(18.85,39.95,0.5,0.5);

	var maskedShapeInstanceList = [this.instance];

	for(var shapedInstanceItr = 0; shapedInstanceItr < maskedShapeInstanceList.length; shapedInstanceItr++) {
		maskedShapeInstanceList[shapedInstanceItr].mask = mask;
	}

	this.timeline.addTween(cjs.Tween.get(this.instance).wait(600));

	// color
	this.filler = new lib.Symbol22();
	this.filler.setTransform(215.35,267.8,1.0108,1.0109,0,0,0,229.8,230.1);

	var maskedShapeInstanceList = [this.filler];

	for(var shapedInstanceItr = 0; shapedInstanceItr < maskedShapeInstanceList.length; shapedInstanceItr++) {
		maskedShapeInstanceList[shapedInstanceItr].mask = mask;
	}

	this.timeline.addTween(cjs.Tween.get(this.filler).wait(600));

	this._renderFirstFrame();

}).prototype = p = new cjs.MovieClip();
p.nominalBounds = new cjs.Rectangle(-16.9,35.2,465,465);


(lib.Symbol7copy = function(mode,startPosition,loop) {
	this.initialize(mode,startPosition,loop,{});

	// Layer_1
	this.instance = new lib.CachedBmp_5();
	this.instance.setTransform(12,5.4,0.5,0.5);

	this.instance_1 = new lib.Symbol17("synched",0);
	this.instance_1.setTransform(19.7,19.7,1,1,0,0,0,19.7,19.7);

	this.instance_2 = new lib.CachedBmp_6();
	this.instance_2.setTransform(12,5.4,0.5,0.5);

	this.instance_3 = new lib.Symbol17copy("synched",0);
	this.instance_3.setTransform(19.7,19.7,1,1,0,0,0,19.7,19.7);

	this.instance_4 = new lib.CachedBmp_7();
	this.instance_4.setTransform(12,7,0.5,0.5);

	this.timeline.addTween(cjs.Tween.get({}).to({state:[{t:this.instance_1},{t:this.instance}]}).to({state:[{t:this.instance_3},{t:this.instance_2}]},1).to({state:[{t:this.instance_3},{t:this.instance_4}]},1).wait(1));

	this._renderFirstFrame();

}).prototype = p = new cjs.MovieClip();
p.nominalBounds = new cjs.Rectangle(-2.1,-2.1,43.5,44);


(lib.Symbol7 = function(mode,startPosition,loop) {
	this.initialize(mode,startPosition,loop,{});

	// Layer_1
	this.instance = new lib.CachedBmp_2();
	this.instance.setTransform(9.8,5.4,0.5,0.5);

	this.instance_1 = new lib.Symbol17("synched",0);
	this.instance_1.setTransform(19.7,19.7,1,1,0,0,0,19.7,19.7);

	this.instance_2 = new lib.CachedBmp_3();
	this.instance_2.setTransform(9.8,5.4,0.5,0.5);

	this.instance_3 = new lib.Symbol17copy("synched",0);
	this.instance_3.setTransform(19.7,19.7,1,1,0,0,0,19.7,19.7);

	this.instance_4 = new lib.CachedBmp_4();
	this.instance_4.setTransform(9.8,7,0.5,0.5);

	this.timeline.addTween(cjs.Tween.get({}).to({state:[{t:this.instance_1},{t:this.instance}]}).to({state:[{t:this.instance_3},{t:this.instance_2}]},1).to({state:[{t:this.instance_3},{t:this.instance_4}]},1).wait(1));

	this._renderFirstFrame();

}).prototype = p = new cjs.MovieClip();
p.nominalBounds = new cjs.Rectangle(-2.1,-2.1,43.5,44);


(lib.Symbol15 = function(mode,startPosition,loop) {
	this.initialize(mode,startPosition,loop,{});

	// Layer_6 (mask)
	var mask = new cjs.Shape();
	mask._off = true;
	mask.graphics.moveTo(-180.3,70.8).curveTo(-203.4,-0.2,-171.8,-67.9).curveTo(-140.4,-135.6,-71.1,-163.6).curveTo(-1.8,-191.7,67.9,-164.9).curveTo(137.5,-138.1,170.3,-71).curveTo(203.1,-3.9,181.4,67.5).curveTo(159.4,138.9,94.8,176.3).lineTo(36.8,76.1).curveTo(62,61.6,70.5,33.7).curveTo(79,5.9,66.4,-20.3).curveTo(53.6,-46.4,26.5,-56.7).curveTo(-0.7,-67.2,-27.6,-56.2).curveTo(-54.7,-45.4,-67,-19.1).curveTo(-79.2,7.3,-70.2,34.9).curveTo(-61.3,62.6,-35.8,76.7).lineTo(-91.9,178).curveTo(-157.2,141.8,-180.3,70.8).closePath();
	mask.setTransform(189.9973,177.462);

	// progress_bar
	this.progressPie = new lib.Symbol14copy();
	this.progressPie.setTransform(-21.25,-42.05);

	var maskedShapeInstanceList = [this.progressPie];

	for(var shapedInstanceItr = 0; shapedInstanceItr < maskedShapeInstanceList.length; shapedInstanceItr++) {
		maskedShapeInstanceList[shapedInstanceItr].mask = mask;
	}

	this.timeline.addTween(cjs.Tween.get(this.progressPie).wait(1));

	// dial_layout
	this.instance = new lib.Symbol13("synched",0);
	this.instance.setTransform(190,190,1,1,0,0,0,190,190);

	this.timeline.addTween(cjs.Tween.get(this.instance).wait(1));

	this._renderFirstFrame();

}).prototype = getMCSymbolPrototype(lib.Symbol15, new cjs.Rectangle(-2,-2,384,384), null);


// stage content:
(lib.igripwidgetv01 = function(mode,startPosition,loop) {
	this.initialize(mode,startPosition,loop,{});

	this.isSingleFrame = false;
	// timeline functions:
	this.frame_0 = function() {
		if(this.isSingleFrame) {
			return;
		}
		if(this.totalFrames == 1) {
			this.isSingleFrame = true;
		}
		var selectorAngle = 0;
		var clearanceLower = 30;
		var clearanceUpper = 330;
		var decimalPrecision = 2;
		
		//this.addEventListener("tick",fl_RotateContinuously.bind(this));
		
		//setup input boxes
		var limitInput;
		var valueInput;
		var widgetName;
		
		var lowerBoundVal = 0.0; //minimum output value
		var upperBoundVal = 100.0; //maximum output value
		
		var displayScaledValue = true; //setting this to false will make the widget use degrees instead
		
		var scaledVal = 0.0; //running value
		var scaledLimit = 0.0; //limit
		var trueScaledVal = 0.0; //indicated by covered portion
		
		var granularity = 0.01;
		var minGranularity = 0.01;
		var maxGranularity = 1;
		
		//this.dial.progressPie.filler.tint = 0.5;
		var defaultDialColor = "#34aeff";
		var dialFiller = null;
		
		this.overlayContainerDiv = "#dom_overlay_container";
		
		this.UUID = uuidv4();
		
		this.setTrueValue = function(val){
			if(val < lowerBoundVal || val > upperBoundVal){
				console.log("Could not set value to: " + val + ". Has to be within [" + lowerBoundVal + ", " + upperBoundVal + "]");
				return;
			}
			
			if(val > scaledLimit){
				console.log("Value cannot be greater than limit: " + scaledLimit);
				console.log("Maximizing to limit!");
				val = scaledLimit;
			}
			
			val = round(val, granularity);
			
			var previousTrueScaledVal = trueScaledVal;
			trueScaledVal = val;
			
			var fromDeg = parseInt(scaledValToDeg(previousTrueScaledVal));
			var toDeg = parseInt(scaledValToDeg(trueScaledVal));
			
			if(fromDeg < toDeg){
				playSegment(this.dial.progressPie, fromDeg - 30, toDeg - 30);
			}else{
				//60 -> 30  === frame 30 -> frame 0 === frame 570 -> frame 600
				playSegment(this.dial.progressPie, 600 - (fromDeg - 30), 600 - (toDeg - 30));
			}
			
			/*if(lastMadeValue < parseInt(this.valueHand.rotation, 10)){
				playSegment(this.dial.progressPie, lastMadeValue - 30, parseInt(this.valueHand.rotation, 10) - 30);
			}else{
				playSegment(this.dial.progressPie, parseInt(this.valueHand.rotation, 10) - 30 + 300, lastMadeValue - 30 + 300);
			}
			
			//this will be determined by feedback received from controller / backend
			lastMadeValue = parseInt(this.valueHand.rotation, 10);*/
		}
		
		this.increaseGranularity = function(){
			//lastDigit = Number.isInteger(sampleNumber) ? sampleNumber % 10
		    //: sampleNumber.toString().slice(-1);
			
			if(granularity < maxGranularity){
				var lastDigit = granularity.toString().slice(-1);
				
				if(lastDigit == 5){
					this.setGranularity(granularity * 2);
				}else{
					this.setGranularity(granularity * 5);
				}
			}else{
				this.setGranularity(maxGranularity);
			}
		}
		
		this.decreaseGranularity = function(){
			var lastDigit = granularity.toString().slice(-1);
			
			if(granularity >= maxGranularity){
				if(lastDigit == 1){
					this.setGranularity(maxGranularity / 2);
				}else{
					this.setGranularity(maxGranularity / 5);
				}
			}else{
				if(granularity <= minGranularity){
					this.setGranularity(minGranularity);
					return;
				}
				
				if(lastDigit == 5){
					this.setGranularity(granularity / 5);
				}else{
					this.setGranularity(granularity / 2);
				}
			}
		}
		
		this.setGranularity = function(val){
			granularity = val;
			this.granularityTxt.text = granularity;
		}
		
		this.setColor = function(val){
			if(dialFiller){
				this.dial.progressPie.filler.removeChild(dialFiller);
			}
			
			dialFiller = new createjs.Shape(new createjs.Graphics().beginFill(val).drawRect(5,5,500,500));
			this.dial.progressPie.filler.addChild(dialFiller);
		}
		
		this.setValue = function(val){
			if(val < lowerBoundVal || val > upperBoundVal){
				console.log("Could not set value to: " + val + ". Has to be within [" + lowerBoundVal + ", " + upperBoundVal + "]");
				return;
			}
			
			if(val > scaledLimit){
				console.log("Value cannot be greater than limit: " + scaledLimit);
				console.log("Maximizing to limit!");
				val = scaledLimit;
			}
			
			val = round(val, granularity);
			
			scaledVal = val;
			this.valueHand.rotation = scaledValToDeg(val);
			$(this.overlayContainerDiv)[0].children["valueInput"].value = displayScaledValue ? scaledVal.toFixed(decimalPrecision) : this.valueHand.rotation.toFixed(decimalPrecision);
			parent.setValueCallback(scaledVal, this.UUID);
		}
		
		this.setLimit = function(val){
			if(val < lowerBoundVal || val > upperBoundVal){
				console.log("Could not set value to: " + val + ". Has to be within [" + lowerBoundVal + ", " + upperBoundVal + "]");
				return;
			}
			
			if(val < scaledVal){
				console.log("Limit cannot be less than current (set) value!");
				console.log("Minimizing to current (set) value!");
				val = scaledVal;
			}
			
			val = round(val, granularity);
		
			scaledLimit = val;
			this.limitHand.rotation = scaledValToDeg(val);
			$(this.overlayContainerDiv)[0].children["limitInput"].value = displayScaledValue ? scaledLimit.toFixed(decimalPrecision) : this.limitHand.rotation.toFixed(decimalPrecision);
		}
		
		// ACCESSORS FOR BOUNDS
		
		this.setLowerBound = function (val) {
			lowerBoundVal = val;
			this.lowerLimitTxt.text = val.toFixed(decimalPrecision);
		}
		
		this.setUpperBound = function (val) {
			upperBoundVal = val;
			this.upperLimitTxt.text = val.toFixed(decimalPrecision);
		}
		
		this.getLowerBound = function () {
			return lowerBoundVal;
		}
		
		this.getUpperBound = function () {
			return upperBoundVal;
		}
		//END ACCESSORS FOR BOUNDS
		
		//gets scaled value b/w lower & upperbound based on user selection
		this.getValue = function () {
			return degToScaledVal(this.valueHand.rotation);
		}
		
		this.getDegrees = function () {
			return this.valueHand.rotation;
		}
		
		//converts degrees to scaled value (value is within lower and upper bounds)
		function degToScaledVal(deg) {
			if (deg < clearanceLower || deg > clearanceUpper) {
				console.log("Invalid input: " + deg + "! Must be b/w [" + clearanceLower + ", " + clearanceUpper + "]");
				return -1;
			}
		
			var fromZeroDeg = deg - clearanceLower; //brings it within [0, clearanceUpper - clearanceLower]
		
			var valPerDeg = ((upperBoundVal - lowerBoundVal) / ((clearanceUpper - clearanceLower)));
		
			return (valPerDeg * fromZeroDeg) + lowerBoundVal;
		}
		
		//converts scaled value to degrees (degrees are within clearanceLower and clearanceUpper)
		function scaledValToDeg(val) {
			if (val < lowerBoundVal || val > upperBoundVal) {
				console.log("Invalid input: " + val + "! Must be b/w [" + lowerBoundVal + ", " + upperBoundVal + "]");
				return -1;
			}
		
			var fromZeroVal = val - lowerBoundVal; //brings it within [0, val - lowerBoundVal]
		
			var valPerDeg = ((upperBoundVal - lowerBoundVal) / ((clearanceUpper - clearanceLower)));
		
			return (1 / valPerDeg) * fromZeroVal + clearanceLower;
		}
		
		function round(value, step) {
		    step || (step = 1.0);
		    var inv = 1.0 / step;
		    return Math.round(value * inv) / inv;
		}
		
		this.setWidgetName = function(name){
			if(name && widgetName){
				widgetName.innerHTML = name;
			}
		}.bind(this);
		
		exportRoot = this;
		
		setTimeout(function () {
			//SETUP WIDGET NAME
			widgetName = $(this.overlayContainerDiv)[0].children["widgetName"];
			widgetName.style.fontSize = "25px";
			widgetName.style.fontFamily = "Burbank Big Wide";
			widgetName.style.fontWeight = "bold";
			widgetName.style.textAlign = "center";
			widgetName.innerHTML = "Position";
		
			//SETUP VALUE INPUT
			valueInput = $(this.overlayContainerDiv)[0].children["valueInput"];
		
			valueInput.style.fontSize = "25px";
			valueInput.style.color = "#000000";
			valueInput.style.fontFamily = "Burbank Big Wide";
			valueInput.style.fontWeight = "bold";
			valueInput.style.border = "none";
			valueInput.style.background = "transparent";
			valueInput.maxLength = 6;
		
			//SETUP LIMIT INPUT
			// $(this.overlayContainerDiv)[0] is a global reference to a div element created by Animate CC
			limitInput = $(this.overlayContainerDiv)[0].children["limitInput"];
		
			limitInput.style.fontSize = "25px";
			limitInput.style.color = "#9A0000";
			limitInput.style.fontFamily = "Burbank Big Wide";
			limitInput.style.fontWeight = "bold";
			limitInput.style.border = "none";
			limitInput.style.background = "transparent";
			limitInput.maxLength = 6;
		
			//anonymous keyhandler to detect enter and push changes thru for limit
			limitInput.onkeypress = function (evt) {
				if (!evt) {
					evt = window.event;
				}
		
				var keyCode = evt.keyCode || evt.which;
		
				//detect enter key
				if (keyCode == '13') {
					console.log("Enter detected -- set limit!");
					//this.limitHand.rotation = displayScaledValue ? scaledValToDeg(limitInput.value) : limitInput.value;
					this.setLimit(displayScaledValue ? limitInput.value : degToScaledVal(limitInput.value));
					return false;
				}
			}.bind(this);
			
			//anonymous keyhandler to detect enter and push changes thru for value
			valueInput.onkeypress = function (evt) {
				if (!evt) {
					evt = window.event;
				}
		
				var keyCode = evt.keyCode || evt.which;
		
				//detect enter key
				if (keyCode == '13') {
					console.log("Enter detected -- set value!");
					console.log("scaledValToDeg(valueInput.value): = " + valueInput.value + " " + scaledValToDeg(valueInput.value));
					//this.valueHand.rotation = displayScaledValue ? scaledValToDeg(valueInput.value) : valueInput.value;
					this.setValue(displayScaledValue ? valueInput.value : degToScaledVal(valueInput.value));
					return false;
				}
			}.bind(this);
			
			this.setColor(defaultDialColor);
			this.setLimit((this.getUpperBound() - this.getLowerBound()) / 2);
			this.setValue((this.getUpperBound() - this.getLowerBound()) / 4);
			
			//SETUP GRANULARITY BTN ROCKERS
			
			this.incrGranularityBtn.addEventListener("click", this.increaseGranularity.bind(this));
			this.decrGranularityBtn.addEventListener("click", this.decreaseGranularity.bind(this));
		
		}.bind(this), 0);
		
		this.limitHand.addEventListener("mousedown", pressLimitHand.bind(this));
		this.limitHand.addEventListener("pressmove", moveLimitHand.bind(this));
		this.limitHand.addEventListener("pressup", releaseLimitHand.bind(this));
		
		this.valueHand.addEventListener("mousedown", pressValueHand.bind(this));
		this.valueHand.addEventListener("pressmove", moveValueHand.bind(this));
		this.valueHand.addEventListener("pressup", releaseValueHand.bind(this));
		
		function pressValueHand(evt) {
			var item = evt.currentTarget;
			item.drag = true;
		}
		
		function playSegment(target, begin, end) {
			console.log("playSegment begin: " + begin + " end: " + end);
			
			if(begin == end){
				return;
			}
			
			if(end < begin){
				console.log("Error end has to be greater than beginning frame!");
				return;
			}
			
			target.alpha = 0.5;
			target.gotoAndPlay(begin);
			target.addEventListener(
				"tick",
				function (e) {
					//console.log("target current frame: " + target.currentFrame);
					if (target.currentFrame == end) {
						target.stop();
						target.removeEventListener("tick", arguments.callee);
						target.alpha = 1;
					}
				}
			);
		}
		
		
		function moveValueHand(evt) {
			var pt = this.dial.globalToLocal(evt.stageX, evt.stageY);
			pt.x -= this.dial.nominalBounds.width / 2;
			pt.y -= this.dial.nominalBounds.height / 2;
		
			console.log("local X: " + pt.x + " local Y: " + pt.y);
		
			var rotationAmount = getHandRotationFromCoord(pt.x, pt.y);
		
			//this.valueHand.rotation = rotationAmount;
			//$(this.overlayContainerDiv)[0].children["valueInput"].value = displayScaledValue ? degToScaledVal(rotationAmount).toFixed(decimalPrecision) : rotationAmount.toFixed(decimalPrecision);
			//this.dial.progressPie.gotoAndStop(rotationAmount - 30);
			this.setValue(degToScaledVal(rotationAmount));
		}
		
		function releaseValueHand(evt) {
			var item = evt.currentTarget;
			item.drag = false;	
		}
		
		function pressLimitHand(evt) {
			var item = evt.currentTarget;
			item.drag = true;
		}
		
		function releaseLimitHand(evt) {
			var item = evt.currentTarget;
			item.drag = false;
		}
		
		function moveLimitHand(evt) {
			//console.log("evt.stageX, evt.stageY: " + evt.stageX + "," + evt.stageY);
			var item = this.limitHand;
		
			var pt = this.dial.globalToLocal(evt.stageX, evt.stageY);
			pt.x -= this.dial.nominalBounds.width / 2;
			pt.y -= this.dial.nominalBounds.height / 2;
		
			var rotationAmount = getHandRotationFromCoord(pt.x, pt.y);
		
			this.setLimit(degToScaledVal(rotationAmount));
		}
		
		function getHandRotationFromCoord(x, y) {
			var rotationAmount = angleFromCoord(x, y);
		
			rotationAmount -= 90; //substract 90 so angle makes sense on the graph
		
			//make positive [0, 360)
			if (rotationAmount < 0) {
				rotationAmount += 360; 
			}
		
			//constrain to the two bounds [30, 330]
			if (rotationAmount > clearanceUpper) {
				rotationAmount = clearanceUpper;
			}
		
			if (rotationAmount < clearanceLower) {
				rotationAmount = clearanceLower;
			}
		
			return rotationAmount;
		}
		
		function angleFromCoord(x, y) {
			var theta = Math.atan2(y, x); // range (-PI, PI]
			theta *= 180 / Math.PI; // rads to degs, range (-180, 180]
		
			if (theta < 0) theta = 360 + theta; // range [0, 360)
		
			return theta;
		}
		
		function uuidv4() {
		  return 'xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx'.replace(/[xy]/g, function(c) {
		    var r = Math.random() * 16 | 0, v = c == 'x' ? r : (r & 0x3 | 0x8);
		    return v.toString(16);
		  });
		}
	}

	// actions tween:
	this.timeline.addTween(cjs.Tween.get(this).call(this.frame_0).wait(1));

	// Actions
	this.widgetName = new lib.an_Label({'id': 'widgetName', 'label':'Label', 'disabled':false, 'visible':true, 'class':'ui-label'});

	this.widgetName.setTransform(130.5,249,1,1,0,0,0,50,11);

	this.timeline.addTween(cjs.Tween.get(this.widgetName).wait(1));

	// lowerLimitTxt
	this.lowerLimitTxt = new cjs.Text("0.00", "25px 'Burbank Big Wd Bd'");
	this.lowerLimitTxt.name = "lowerLimitTxt";
	this.lowerLimitTxt.textAlign = "center";
	this.lowerLimitTxt.lineHeight = 27;
	this.lowerLimitTxt.lineWidth = 119;
	this.lowerLimitTxt.parent = this;
	this.lowerLimitTxt.setTransform(41.85,211.2,0.5677,0.5677);

	this.timeline.addTween(cjs.Tween.get(this.lowerLimitTxt).wait(1));

	// upperLimitTxt
	this.upperLimitTxt = new cjs.Text("100.00", "25px 'Burbank Big Wd Bd'");
	this.upperLimitTxt.name = "upperLimitTxt";
	this.upperLimitTxt.textAlign = "center";
	this.upperLimitTxt.lineHeight = 27;
	this.upperLimitTxt.lineWidth = 119;
	this.upperLimitTxt.parent = this;
	this.upperLimitTxt.setTransform(222.45,211.2,0.5677,0.5677);

	this.timeline.addTween(cjs.Tween.get(this.upperLimitTxt).wait(1));

	// Layer_91
	this.decrGranularityBtn = new lib.Symbol7copy();
	this.decrGranularityBtn.setTransform(145.35,188.5,0.5677,0.5677,0,0,0,19.8,19.8);
	new cjs.ButtonHelper(this.decrGranularityBtn, 0, 1, 2);

	this.granularityTxt = new cjs.Text("0.01", "30px 'Burbank Big Wd Md'");
	this.granularityTxt.name = "granularityTxt";
	this.granularityTxt.textAlign = "center";
	this.granularityTxt.lineHeight = 36;
	this.granularityTxt.lineWidth = 111;
	this.granularityTxt.parent = this;
	this.granularityTxt.setTransform(129.4886,116.95,0.5677,0.5677);

	this.instance = new lib.CachedBmp_1();
	this.instance.setTransform(110.5,96.5,0.5,0.5);

	this.incrGranularityBtn = new lib.Symbol7();
	this.incrGranularityBtn.setTransform(115.55,188.5,0.5677,0.5677,0,0,0,19.8,19.8);
	new cjs.ButtonHelper(this.incrGranularityBtn, 0, 1, 2);

	this.timeline.addTween(cjs.Tween.get({}).to({state:[{t:this.incrGranularityBtn},{t:this.instance},{t:this.granularityTxt},{t:this.decrGranularityBtn}]}).wait(1));

	// setValueHand
	this.valueHand = new lib.Symbol16();
	this.valueHand.setTransform(130.4,112.5,0.5677,0.5677,89.7336,0,0,0.5,-75);
	new cjs.ButtonHelper(this.valueHand, 0, 1, 1);

	this.timeline.addTween(cjs.Tween.get(this.valueHand).wait(1));

	// dial_layout
	this.text = new cjs.Text("Position", "25px 'Burbank Big Wd Bd'");
	this.text.textAlign = "center";
	this.text.lineHeight = 27;
	this.text.lineWidth = 141;
	this.text.parent = this;
	this.text.setTransform(-72.7,277.5);

	this.limitHand = new lib.Symbol16copy();
	this.limitHand.setTransform(130.25,114.35,0.5677,0.5677,-90,0,0,-0.2,-75);
	new cjs.ButtonHelper(this.limitHand, 0, 1, 1);

	this.dial = new lib.Symbol15();
	this.dial.setTransform(130.35,112.45,0.5677,0.5677,0,0,0,190.2,190.1);

	this.timeline.addTween(cjs.Tween.get({}).to({state:[{t:this.dial},{t:this.limitHand},{t:this.text}]}).wait(1));

	this._renderFirstFrame();

}).prototype = p = new cjs.MovieClip();
p.nominalBounds = new cjs.Rectangle(-13.3,132.7,277.90000000000003,171.8);
// library properties:
lib.properties = {
	id: 'C6E33BD7FF3D4B13AB357016EFED1C8E',
	width: 264,
	height: 264,
	fps: 60,
	color: "#FFFFFF",
	opacity: 1.00,
	manifest: [
		{src:"images/igrip_widget_v0.1_atlas_.png?1576714212645", id:"igrip_widget_v0.1_atlas_"},
		{src:"components/lib/jquery-3.4.1.min.js?1576714212714", id:"lib/jquery-3.4.1.min.js"},
		{src:"components/sdk/anwidget.js?1576714212714", id:"sdk/anwidget.js"},
		{src:"components/ui/src/textinput.js?1576714212714", id:"an.TextInput"},
		{src:"components/ui/src/label.js?1576714212714", id:"an.Label"}
	],
	preloads: []
};



// bootstrap callback support:

(lib.Stage = function(canvas) {
	createjs.Stage.call(this, canvas);
}).prototype = p = new createjs.StageGL();

p.setAutoPlay = function(autoPlay) {
	this.tickEnabled = autoPlay;
}
p.play = function() { this.tickEnabled = true; this.getChildAt(0).gotoAndPlay(this.getTimelinePosition()) }
p.stop = function(ms) { if(ms) this.seek(ms); this.tickEnabled = false; }
p.seek = function(ms) { this.tickEnabled = true; this.getChildAt(0).gotoAndStop(lib.properties.fps * ms / 1000); }
p.getDuration = function() { return this.getChildAt(0).totalFrames / lib.properties.fps * 1000; }

p.getTimelinePosition = function() { return this.getChildAt(0).currentFrame / lib.properties.fps * 1000; }

an.bootcompsLoaded = an.bootcompsLoaded || [];
if(!an.bootstrapListeners) {
	an.bootstrapListeners=[];
}

an.bootstrapCallback=function(fnCallback) {
	an.bootstrapListeners.push(fnCallback);
	if(an.bootcompsLoaded.length > 0) {
		for(var i=0; i<an.bootcompsLoaded.length; ++i) {
			fnCallback(an.bootcompsLoaded[i]);
		}
	}
};

an.compositions = an.compositions || {};
an.compositions['C6E33BD7FF3D4B13AB357016EFED1C8E'] = {
	getStage: function() { return exportRoot.stage; },
	getLibrary: function() { return lib; },
	getSpriteSheet: function() { return ss; },
	getImages: function() { return img; }
};

an.compositionLoaded = function(id) {
	an.bootcompsLoaded.push(id);
	for(var j=0; j<an.bootstrapListeners.length; j++) {
		an.bootstrapListeners[j](id);
	}
}

an.getComposition = function(id) {
	return an.compositions[id];
}


an.makeResponsive = function(isResp, respDim, isScale, scaleType, domContainers) {		
	var lastW, lastH, lastS=1;		
	window.addEventListener('resize', resizeCanvas);		
	resizeCanvas();		
	function resizeCanvas() {			
		var w = lib.properties.width, h = lib.properties.height;			
		var iw = window.innerWidth, ih=window.innerHeight;			
		var pRatio = window.devicePixelRatio || 1, xRatio=iw/w, yRatio=ih/h, sRatio=1;			
		if(isResp) {                
			if((respDim=='width'&&lastW==iw) || (respDim=='height'&&lastH==ih)) {                    
				sRatio = lastS;                
			}				
			else if(!isScale) {					
				if(iw<w || ih<h)						
					sRatio = Math.min(xRatio, yRatio);				
			}				
			else if(scaleType==1) {					
				sRatio = Math.min(xRatio, yRatio);				
			}				
			else if(scaleType==2) {					
				sRatio = Math.max(xRatio, yRatio);				
			}			
		}			
		domContainers[0].width = w * pRatio * sRatio;			
		domContainers[0].height = h * pRatio * sRatio;			
		domContainers.forEach(function(container) {				
			container.style.width = w * sRatio + 'px';				
			container.style.height = h * sRatio + 'px';			
		});			
		stage.scaleX = pRatio*sRatio;			
		stage.scaleY = pRatio*sRatio;			
		lastW = iw; lastH = ih; lastS = sRatio;            
		stage.tickOnUpdate = false;            
		stage.update();            
		stage.tickOnUpdate = true;		
	}
}
function _updateVisibility(evt) {
	if((this.stage == null || this._off || this._lastAddedFrame != this.parent.currentFrame) && this._element) {
		this._element.detach();
		stage.removeEventListener('drawstart', this._updateVisibilityCbk);
		this._updateVisibilityCbk = false;
	}
}
function _handleDrawEnd(evt) {
	var props = this.getConcatenatedDisplayProps(this._props), mat = props.matrix;
	var tx1 = mat.decompose(); var sx = tx1.scaleX; var sy = tx1.scaleY;
	var dp = window.devicePixelRatio || 1; var w = this.nominalBounds.width * sx; var h = this.nominalBounds.height * sy;
	mat.tx/=dp;mat.ty/=dp; mat.a/=(dp*sx);mat.b/=(dp*sx);mat.c/=(dp*sy);mat.d/=(dp*sy);
	this._element.setProperty('transform-origin', this.regX + 'px ' + this.regY + 'px');
	var x = (mat.tx + this.regX*mat.a + this.regY*mat.c - this.regX);
	var y = (mat.ty + this.regX*mat.b + this.regY*mat.d - this.regY);
	var tx = 'matrix(' + mat.a + ',' + mat.b + ',' + mat.c + ',' + mat.d + ',' + x + ',' + y + ')';
	this._element.setProperty('transform', tx);
	this._element.setProperty('width', w);
	this._element.setProperty('height', h);
	this._element.update();
}

function _tick(evt) {
	var stage = this.stage;
	stage&&stage.on('drawend', this._handleDrawEnd, this, true);
	if(!this._updateVisibilityCbk) {
		this._updateVisibilityCbk = stage.on('drawstart', this._updateVisibility, this, false);
	}
}


})(createjs = createjs||{}, AdobeAn = AdobeAn||{});
var createjs, AdobeAn;