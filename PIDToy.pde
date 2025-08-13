 //<>// //<>//
float wheelposmax = 2.0; //in revolutions from zero, hard stop (speed to zero and does not allow past)
float endstoprestitution = 0.1;
float displayscale = 0.4; //size of window x this - 0.4 for rad 1.0 (diameter 2.0) with margin
float simwindowwidth = 0.6; // processing window width x this
int infowidthmin = 320; // px absolute minimum info panel width
float setpointindicatorsize = 0.05;
float setpointindicatordist = 0.1;
float setpointdragdivider = 10;
float sliderheight = 16.0;
float maxpidval = 10.0;
float maxmotortorqueval = 100.0;
float maxmotorspeedval = 64 * TWO_PI;
float minmotorspeedval = QUARTER_PI;
float maxwheelradius = 2.0;
float minwheelradius = 0.1;
float minwheelmass = 0.1; //kg
float maxwheelmass = 20.0;

float scrollwheelmult = 0.9; // multiplies/divides value by this on scroll over slider

//history length, for graphs:
int historysamplesmax = 3000; //samples. 1 sample per PID/sim loop
int historytimeframemax = 10; //seconds

//SIM VARS
float simtimescale = 1.0; // simsec/sec
float simfreq = 300.0; //Hz
float axlefriction = 0.0001; //Nm dynamic
float maxaxlefrictionval = 1.0;
float wheelradius = 1.0; //meter
float wheelmass = 0.5; //kg
float motorstalltorque = 1.0; //Stall torque, Nm, fades off linear to 0 at unloadedspeed
float motorpower = 10.0; //watts, currently unused
float motorunloadedspeed = 16*TWO_PI; //rad/s, * twopi for rot/s
volatile float motorout = 0.0; //output power, -1 to 1
volatile float motortorque = 0; //motorpower * motorout = nm torque out
volatile float frictiontorque = 0; //opposes motion with axlefriction
volatile float wheelpos = 0.0; //angle from 0 (vertical) in rads
volatile float wheelvel = 0; // rad/s
volatile float wheelaccel = 0; // rads/s^2
volatile float setpointpos = 0.0; //rads
volatile float Pc = 0.0, Pp = 0.0, Ic = 0.0, Dc = 0.0;
volatile float externaltorque = 0;
volatile float wheelmoment = 1.0;
volatile float springforce = 0.0;
volatile float springangle = 0.0;

float externalforcemult = 100.0; //N/m, effectively spring constant

ArrayList<float[]> loophistory; // [P, I, D, KP, KI, KD, Out, PowerOut, Torque, vel, Pos, setpoint, simframe, ns_timestamp]

volatile float KP = 1.0, KI = 1.0, KD = 1.0; //PID'z nutz!

float dsx = 0.0, dsy = 0.0; //drag start x/y (starting pointer loc in sim coordinates)
float dsr = 1.0, dst = 0.0; //drag start radius and theta on wheel, sim scale
float dcx = 0.0, dcy = 0.0; //drag pos x/y (current pointer loc in sim coordinates)
float dragang = 0.0; //current angle in radians from vertical, about 0,0; sim coords

volatile boolean applymouseforce = false;
volatile boolean draggingsetpoint = false;
boolean draggingslider = false;
int sliderindex = 0;
float sliderdragpos = 0.0;

boolean siminfotext = true;
boolean simdbg = true;

volatile boolean killsim = false; //stop the sim by seting this true
volatile boolean simrunning = false; //verify sim start/stop - turns false after scussful kill
volatile float simframerate = 0.0;
volatile long simframes = 0;
long simstartns = 0;

//UI
color simbackcolor = color(16, 16, 16, 255);
color framecolor = color(100, 100, 100, 255);
color wheelcolor = color(64, 64, 64, 255);
color pointercolor = color(240, 240, 240, 255); // position indicator line on wheel
color wheeloutlinecolor = color(127, 127, 127, 255);
color setpointcolor = color(255, 255, 255, 255);
color setpointdraglinecolor = color(220, 64, 64, 200);
color sliderbackcolor = color(16, 16, 16, 255);

color graphgridcolor = color(127, 127, 127, 127);
color pcolor = color(220, 32, 32, 240);
color icolor = color(32, 220, 32, 240);
color dcolor = color(32, 32, 220, 240);
color ocolor = color(127, 127, 127, 200);

void setup() {
  size(1024, 720, P2D);
  smooth(8);
  frameRate(60);
  loophistory = new ArrayList<float[]>();
  thread("simthread");
}

void draw() {
  simsync();
  drawbackground();
  drawsimwindow();
  if (draggingsetpoint) {
    drawdragline(true);
  } else if (applymouseforce) {
    drawdragline(false);
  }
  drawgraphwindow();
  drawframe();
  drawcontrols();
  if (mousePressed) {
    drawsimcursor();
  }
  if (frameCount%60 == 0) {
    println(nf(simframes, 0, 0) + " sim frames");
    println(nf(getavgsimfps(), 0, 3) + " sim FPS");
  }
}

void simthread() { //main thread for physics, to run independent of draw
  long tns = System.nanoTime();
  simstartns = tns;
  long simdiv = floor(1000000000.0/simfreq); //delta t
  if (simdbg) {
    print(nf(tns, 16, 0));
    println(" Sim Thread Started");
  }
  try {
    simrunning = true;
    while (!killsim) {
      //MAIN SIM LOOP START
      while (tns + simdiv > System.nanoTime()) {
        delay(1); //one ms. not super precise, oops
        //Thread.sleep(0, 255);
      }

      if (draggingsetpoint) {
        setpointpos += ((dragang-setpointpos)*setpointdragdivider)/simfreq;
      }

      tns += simdiv; //catch up on frames if behind instead of dropping....? maybe limit to a certain amount. idk

      Pc = setpointpos - wheelpos;
      Ic = Ic + (Pc / simfreq);
      Dc = (Pc - Pp) * simfreq;
      Pp = Pc;

      motorout = max(-1, min(1, KP*Pc + KI*Ic + KD*Dc)); //range-limited control output of PID loop, full ccw to full cw power

      motortorque = max(0, min(motorstalltorque, motorstalltorque * ((motorunloadedspeed - abs(wheelvel)) / motorunloadedspeed) )) * motorout; //torque from motor power, given unloaded rad/s and stall Nm
      frictiontorque = (wheelvel > 0 ? -min(axlefriction, wheelvel) : (wheelvel < 0 ? max(axlefriction, wheelvel) : 0));

      if (applymouseforce) {
        float springanchorx = dsr*sin(dst+wheelpos);
        float springanchory = -dsr*cos(dst+wheelpos);
        float springangle = atan2(dcy - springanchory, dcx - springanchorx);
        springforce = abs(dist(springanchorx, springanchory, dcx, dcy))*externalforcemult;
        externaltorque = springforce*cos((dst + wheelpos) - springangle);
      } else {
        externaltorque = 0;
      }

      wheelmoment = wheelmass * (wheelradius * wheelradius);
      wheelaccel = (motortorque + frictiontorque + externaltorque) / wheelmoment;
      wheelvel += (wheelaccel / simfreq);
      wheelpos += (wheelvel / simfreq);

      if (abs(wheelpos)>=wheelposmax*TWO_PI) {
        wheelpos = max(-wheelposmax*TWO_PI, min(wheelposmax*TWO_PI, wheelpos));
        wheelvel = -wheelvel*endstoprestitution;
      }

      float[] datum = new float[]{
        Pc, Ic, Dc,
        KP, KI, KD,
        motorout, motortorque, frictiontorque, externaltorque,
        wheelaccel, wheelvel, wheelpos, setpointpos,
        wheelmass, wheelradius, motorpower,
        simframes, (tns-simstartns)};

      loophistory.add(datum);
      simframes++;

      //MAIN SIM LOOP END
    }
  }
  catch(Exception e) {
  }
  if (simdbg) {
    print(nf(tns, 16, 0));
    println(" Sim Thread Stopping");
  }
  simrunning = false;
}

void simsync() {
}

void drawbackground() {
  background(simbackcolor);
}

void drawsimwindow() {
  drawwheel();
  drawsetpoint();
  if (siminfotext) {
    drawinfotext();
  }
}

void drawsimcursor() {
  float simx = screentosimx(mouseX);
  float simy = screentosimy(mouseY);
  float cx = simtoscreenx(simx);
  float cy = simtoscreeny(simy);
  stroke(color(255, 255, 255, 127));
  strokeWeight(1);
  line(cx - 16, cy, cx + 16, cy);
  line(cx, cy - 16, cx, cy + 16);
}

void drawgraphwindow() {
  erasehistory();
  drawPIDgraph();
}

void erasehistory() { //dangerous, do NOT use on actual textbooks
  while (loophistory.size()>historysamplesmax) {
    loophistory.remove(0);
  }
}

float getavgsimfps() {
  float[] datain = new float[19];
  float avgfps = 0.0;
  int samplenum = 0;
  /*
  datum = new float[]{
   0 Pc, 1 Ic, 2 Dc,
   3 KP, 4 KI, 5 KD,
   6 motorout, 7 motortorque, 8 frictiontorque, 9 externaltorque,
   10 wheelaccel, 11 wheelvel, 12 wheelpos, 13 setpointpos,
   14 wheelmass, 15 wheelradius, 16 motorpower,
   17 simframes, 18 tns};
   */
  long cts = 1; //current samples' timestamp
  long pts = 0; //prev timestamp
  datain = loophistory.get(0);
  cts = int(datain[18]);
  for (int i=1; i<min(loophistory.size(), historysamplesmax); i++) {
    try {
      pts = cts;
      datain = loophistory.get(i);
      cts = (long)datain[18];
      avgfps += (1000000000 / abs(cts-pts));
      samplenum++;
    }
    catch(Exception e) {
      println(" fps calc failed");
    }
  }
  return avgfps/ (float)samplenum; //<>//
}

void drawPIDgraph() {
  float Pp = 0.0, Ip = 0.0, Dp = 0.0, Op = 0.0;
  float[] datain = new float[19];
  /*
  datum = new float[]{
   0 Pc, 1 Ic, 2 Dc,
   3 KP, 4 KI, 5 KD,
   6 motorout, 7 motortorque, 8 frictiontorque, 9 externaltorque,
   10 wheelaccel, 11 wheelvel, 12 wheelpos, 13 setpointpos,
   14 wheelmass, 15 wheelradius, 16 motorpower,
   17 simframes, 18 tns};
   */
  strokeWeight(1);
  stroke(graphgridcolor);
  line(0, getgraphwindowheighthalf(), getgraphwindowwidth(), getgraphwindowheighthalf());
  for (int i=0; i<min(loophistory.size(), historysamplesmax); i++) {
    try {
      datain = loophistory.get(i);
      strokeWeight(1.5);
      stroke(pcolor);

      float xa = (float(i)/min(loophistory.size(), historysamplesmax))*(width - simscreenwidth());
      float xb = ((float(i)+1)/min(loophistory.size(), historysamplesmax))*(width - simscreenwidth());
      float ya = getgraphwindowheighthalf() + (Pp * displayscale * (getgraphwindowheight()));
      float yb = getgraphwindowheighthalf() + (datain[0] * displayscale * (getgraphwindowheight()));
      line(xa, ya, xb, yb);

      stroke(icolor);

      ya = getgraphwindowheighthalf() + (Ip * displayscale * (getgraphwindowheight()));
      yb = getgraphwindowheighthalf() + (datain[1] * displayscale * (getgraphwindowheight()));
      line(xa, ya, xb, yb);

      stroke(dcolor);

      ya = getgraphwindowheighthalf() + (Dp * displayscale * (getgraphwindowheight()));
      yb = getgraphwindowheighthalf() + (datain[2] * displayscale * (getgraphwindowheight()));
      line(xa, ya, xb, yb);

      stroke(ocolor);

      ya = getgraphwindowheighthalf() + (Op * displayscale * (getgraphwindowheight()));
      yb = getgraphwindowheighthalf() + (datain[6] * displayscale * (getgraphwindowheight()));
      line(xa, ya, xb, yb);

      Pp = datain[0];
      Ip = datain[1];
      Dp = datain[2];
      Op = datain[6];
    }
    catch(Exception e) {
    }
  }
}

float getgraphwindowwidth() {
  return (width - simscreenwidth());
}

float getgraphwindowheighthalf() {
  return height/4;
}

float getgraphwindowheight() {
  return height/2;
}

void drawframe() {
  drawsimframe();
  drawgraphframe();
}

void drawsimframe() { //box that the simulation lives in, it's not big but it's something
  strokeWeight(1);
  stroke(framecolor);
  noFill();
  rect(width-simscreenwidth(), 0, simscreenwidth()-1, height-1);
}

void drawgraphframe() {
  strokeWeight(1);
  stroke(framecolor);
  noFill();
  rect(0, 0, width-simscreenwidth(), getgraphwindowheighthalf()*2);
}

void drawcontrols() {
  drawfillbox();
  drawPIDsliders();
  drawmechanicalsliders();
  drawsimsliders();
}

void drawfillbox() {
}

void drawslider(float val, color slidercolor, float xp, float yp, float wid, float hig, float minval, float maxval, String label, String unit) { //value, topleft x,y, width, height, min, max
  strokeWeight(1);
  stroke(slidercolor);
  fill(sliderbackcolor);
  rect(xp, yp, wid, hig-1);
  fill(slidercolor);
  rect(map(0, minval, maxval, xp, wid), yp, map(val, minval, maxval, xp, wid), hig-1);
  noStroke();
  fill(255);
  textSize(hig);
  text(label + ": " + nfs(val, 0, 0) + unit, wid/2-textWidth(label + ": " + nfs(val, 0, 0) + unit)/2, yp+hig-2);
}

void drawPIDsliders() {
  drawslider(KP, pcolor, 0.0, (float)(getgraphwindowheighthalf()*2.0), (float)(width-(width*simwindowwidth)), sliderheight, 0, maxpidval, "KP", ""); //value, color, topleft x,y, width, height, min, max
  drawslider(KI, icolor, 0.0, (float)(getgraphwindowheighthalf()*2.0) + sliderheight, (float)(width-(width*simwindowwidth)), sliderheight, 0, maxpidval, "KI", "");
  drawslider(KD, dcolor, 0.0, (float)(getgraphwindowheighthalf()*2.0) + sliderheight*2, (float)(width-(width*simwindowwidth)), sliderheight, 0, maxpidval, "KD", "");
}

void drawmechanicalsliders() {
  drawslider(motorstalltorque, color(127, 240, 240, 255), 0.0, (float)(getgraphwindowheighthalf()*2.0) + sliderheight*3, sliderswidth(), sliderheight, 0, maxmotortorqueval, "Stall T", "Nm");
  drawslider(motorunloadedspeed, color(127, 240, 127, 255), 0.0, (float)(getgraphwindowheighthalf()*2.0) + sliderheight*4, sliderswidth(), sliderheight, 0, maxmotorspeedval, "Unloaded AV", "rad/s");
  drawslider(axlefriction, color(240, 127, 127, 255), 0.0, (float)(getgraphwindowheighthalf()*2.0) + sliderheight*5, sliderswidth(), sliderheight, 0, maxaxlefrictionval, "Axle F", "Nm");
  drawslider(wheelmass, color(220, 220, 220, 255), 0.0, (float)(getgraphwindowheighthalf()*2.0) + sliderheight*6, sliderswidth(), sliderheight, minwheelmass, maxwheelmass, "Wheel Mass", "Kg");
  drawslider(wheelradius, color(240, 200, 220, 255), 0.0, (float)(getgraphwindowheighthalf()*2.0) + sliderheight*7, sliderswidth(), sliderheight, minwheelradius, maxwheelradius, "Wheel Radius", "m");
}

void drawsimsliders() {
}

float sliderswidth() {
  return (width-(width*simwindowwidth));
}

void drawdragline(boolean issetpoint) {
  float sx = 0.0;
  float sy = 0.0;
  if (issetpoint) {
    sx = getsetpointscreenx();
    sy = getsetpointscreeny();
  } else {
    sx = simcenterscreenx() + simtoscreenscale(dsr*sin(dst+wheelpos));
    sy = simcenterscreeny() + simtoscreenscale(-dsr*cos(dst+wheelpos));
  }
  strokeWeight(1.0);
  stroke(setpointdraglinecolor);
  line(mouseX, mouseY, sx, sy);
}

float getsetpointscreenx() {
  return simcenterscreenx() + (height * displayscale * (wheelradius+(setpointindicatordist)))*sin(setpointpos);
}

float getsetpointscreeny() {
  return simcenterscreeny() - (height * displayscale * (wheelradius+(setpointindicatordist)))*cos(setpointpos);
}

void drawwheel() {
  ellipseMode(RADIUS);
  fill(wheelcolor);
  stroke(wheeloutlinecolor);
  strokeWeight(1.5);
  float xc = simcenterscreenx();
  float yc = simcenterscreeny();
  float xe = xc + (simtoscreenscale(wheelradius))*sin(wheelpos); //end of the line
  float ye = yc - (simtoscreenscale(wheelradius))*cos(wheelpos);
  ellipse(xc, yc, wheelradius * displayscale * height, wheelradius * displayscale * height);
  stroke(pointercolor);
  line(xc, yc, xe, ye);
}

float simtoscreenscale(float simsize) {
  return height * displayscale * simsize;
}

void drawsetpoint() {
  float xc = simcenterscreenx();
  float yc = simcenterscreeny();
  float xe = xc + (height * displayscale * (wheelradius+setpointindicatordist))*sin(setpointpos); //end of the line
  float ye = yc - (height * displayscale * (wheelradius+setpointindicatordist))*cos(setpointpos);
  stroke(setpointcolor);
  fill(setpointcolor);
  strokeWeight(1.5);
  ellipse(xe, ye, (height * displayscale * setpointindicatorsize), (height * displayscale * setpointindicatorsize));
}

void drawinfotext() {
}

float simcenterscreenx() {
  return width - (width * simwindowwidth * 0.5);
}

float simcenterscreeny() {
  return height * 0.5;
}

void mousePressed() {
  if (scoordsinsim(mouseX, mouseY)) { //INSIDE SIM REGION
    draggingslider = false;
    dsx = screentosimx(mouseX);
    dsy = screentosimy(mouseY);
    dcx = screentosimx(mouseX);
    dcy = screentosimy(mouseY);
    if (scoordsonwheel(mouseX, mouseY)) {
      applymouseforce = true;
      draggingsetpoint = false;
      dsr = screentosimradius(mouseX, mouseY);
      dst = getdragang(screentosimy(mouseY), screentosimx(mouseX)) - wheelpos;
    } else if (scoordsonsetpoint(mouseX, mouseY)) {
      applymouseforce = false;
      draggingsetpoint = true;
      dragang = getdragang(screentosimy(mouseY), screentosimx(mouseX));
    } else {
      applymouseforce = false;
      draggingsetpoint = false;
    }
    //END OF INSIDE SIM REGION
  } else if (scoordsonsliders(mouseX, mouseY)) { //IN SLIDERS AREA
    applymouseforce = false;
    draggingsetpoint = false;
    draggingslider = true;
    sliderindex = getslidernumbyscreencoord(mouseY);
    sliderdragpos = mouseX / (float)getsliderwidth();
    setslidervalbypos(sliderindex, sliderdragpos);
  } else { //NO IDEA WHERE THE MOUSE IS MATE
    //CAT GOT IT MAYBE?
  }
}

float screentosimradius(float sx, float sy) {
  float rad = 1.0;
  rad = dist(screentosimx(sx), screentosimy(sy), 0, 0);
  return rad;
}

float screentosimtheta(float sx, float sy) {
  float theta = 0.0;
  theta = atan2(screentosimy(sy), screentosimx(sx)) + HALF_PI; // (screentosimx(sx), -screentosimy(sy)) ?
  return theta;
}

boolean scoordsonsliders(float inx, float iny) {
  return inx <= getsliderwidth() && iny >= getgraphwindowheighthalf()*2.0;
}

float getslidervalbyscreencoord(float inx, float iny) { //get the value the slider would be at the input position onscreen
  int slidernum = getslidernumbyscreencoord(iny);
  float sliderpos = (float)inx/ (float)getsliderwidth();
  return getslidervalbyindex(slidernum, sliderpos);
}

int getslidernumbyscreencoord(float screeny) {
  return floor((screeny - getgraphwindowheighthalf()*2.0) / sliderheight);
}

float getsliderwidth() {
  return (width-(width*simwindowwidth));
}

void mouseDragged() {
  if (draggingsetpoint) {
    dcx = screentosimx(mouseX);
    dcy = screentosimy(mouseY);
    dragang = getdragang(dcy, dcx);
  } else if (draggingslider) {
    sliderdragpos = mouseX / (float)getsliderwidth();
    setslidervalbypos(sliderindex, sliderdragpos);
  } else if (applymouseforce) {
    dcx = screentosimx(mouseX);
    dcy = screentosimy(mouseY);
  }
}

void mouseWheel(MouseEvent event) {
  if (scoordsonsliders(mouseX, mouseY)) {
    float wheelcount = event.getCount();
    int i = getslidernumbyscreencoord(mouseY);
    float oldval = getsliderval(i);
    float newval = (wheelcount < 0 ? oldval/scrollwheelmult : oldval*scrollwheelmult);
    print(oldval);
    print(" ");
    println(newval);
    setsliderval(i, newval);
  }
}

float getslidervalbyindex(int index, float pos) {
  float min = 0.0;
  float max = 1.0;
  switch(index) {
  case 0:
  case 1:
  case 2:
    min = 0;
    max = maxpidval;
    break;
  case 3: //stall t
    min=0;
    max = maxmotortorqueval;
    break;
  case 4: //unloaded speed
    min = minmotorspeedval;
    max = maxmotorspeedval;
    break;
  case 5: //axle F
    min = 0;
    max = maxaxlefrictionval;
    break;
  case 6: //wheel mass
    min = minwheelmass;
    max = maxwheelmass;
    break;
  case 7: //wheel radius
    min = minwheelradius;
    max = maxwheelradius;
    break;
  default:
    break;
  }
  return max(min, min(map(pos, 0, 1, min, max), max));
}

void setslidervalbypos(int i, float dpos) {
  switch(i) {
  case 0:
    KP = getslidervalbyindex(i, dpos);
    break;
  case 1:
    KI = getslidervalbyindex(i, dpos);
    break;
  case 2:
    KD = getslidervalbyindex(i, dpos);
    break;
  case 3:
    motorstalltorque = getslidervalbyindex(i, dpos);
    break;
  case 4:
    motorunloadedspeed = getslidervalbyindex(i, dpos);
    break;
  case 5:
    axlefriction = getslidervalbyindex(i, dpos);
    break;
  case 6:
    wheelmass = getslidervalbyindex(i, dpos);
    break;
  case 7:
    wheelradius = getslidervalbyindex(i, dpos);
    break;
  default:
    break;
  }
}

void setsliderval(int i, float inval) {
  switch(i) {
  case 0:
    KP = inval;
    break;
  case 1:
    KI = inval;
    break;
  case 2:
    KD = inval;
    break;
  case 3:
    motorstalltorque = inval;
    break;
  case 4:
    motorunloadedspeed = inval;
    break;
  case 5:
    axlefriction = inval;
    break;
  case 6:
    wheelmass = inval;
    break;
  case 7:
    wheelradius = inval;
    break;
  default:
    break;
  }
}

float getsliderval(int i) {
  float out = 0.0;
  switch(i) {
  case 0:
    out = KP;
    break;
  case 1:
    out = KI;
    break;
  case 2:
    out = KD;
    break;
  case 3:
    out = motorstalltorque;
    break;
  case 4:
    out = motorunloadedspeed;
    break;
  case 5:
    out = axlefriction;
    break;
  case 6:
    out = wheelmass;
    break;
  case 7:
    out = wheelradius;
    break;
  default:
    break;
  }
  return out;
}

float getdragang(float iny, float inx) { //sim coords
  return atan2(iny, inx) + HALF_PI; //it's only a func because of this dumb half pi urgh
}

boolean scoordsinsim(float inx, float iny) {
  return (inx <= width) && (inx > width - simscreenwidth()) && (iny<=height) && (iny>=0);
}

int simscreenwidth() {
  return int(width * simwindowwidth);
}

float screencoordstosimang(float inx, float iny) {
  float angleout = 0.0;
  angleout = atan2(screentosimy(iny), screentosimx(inx)) + HALF_PI;
  return angleout;
}

boolean scoordsonwheel(float mx, float my) {
  float cx = screentosimx(mx);
  float cy = screentosimy(my);
  return (dist(0, 0, cx, cy) <= wheelradius);
}

boolean scoordsonsetpoint(float mx, float my) {
  float cx = screentosimx(mx);
  float cy = screentosimy(my);
  float dist = dist(0, 0, cx, cy);
  return (dist > wheelradius && dist <= wheelradius + setpointindicatordist + setpointindicatorsize);
}

float screentosimx(float inx) {
  return (inx - simcenterscreenx()) / (height * displayscale);
}

float screentosimy(float iny) {
  return (iny - simcenterscreeny()) / (height * displayscale);
}

float simtoscreenx(float inx) {
  return simcenterscreenx() + simtoscreenscale(inx);
}

float simtoscreeny(float iny) {
  return simcenterscreeny() + simtoscreenscale(iny);
}


void mouseReleased() {
  applymouseforce = false;
  draggingsetpoint = false;
  draggingslider = false;
}

void keyReleased() {
  if (key == 'x') {
    endprogram();
  }
}

void endprogram() {
  killsim = true;
  while (simrunning) {
    delay(16);
  }
  if (simdbg) {
    println();
    print(nf(System.nanoTime(), 16, 0));
    println(" Sim Thread Stopped");
  }
  stop();
  exit();
}
//original by John Carmichael
