"strict";

const GRIDMAP = (function gridmap() {
  // Private variables and functions
  var containerDiv;
  // Offscreen canvas
  let width;
  let height;
  const ofsCanvas = document.createElement('canvas');
  const ofsCtx = ofsCanvas.getContext('2d');
  // Onscreen canvas
  const canvasDiv = document.createElement("div");
  const onsCanvas = document.createElement("canvas");
  const onsCtx = onsCanvas.getContext('2d');

  const slider = document.createElement("input");
  const zoomSpan = document.createElement("span");
  const infoDiv = document.createElement("div");
  const infoSpan = document.createElement("span");


  const btnGridsJustLogged = document.createElement("button");

  const projection = proj4('+proj=robin +ellps=WGS84 +datum=WGS84 +pm=oslo +units=dd');
  const img = new Image();

  img.crossOrigin = "anonymous";
  img.src = "./rob4200.png";
  
  const kx = projection.forward([180, 0])[0]; // 17005833.33;
  const ky = projection.forward([0, 90])[1];  //  8625154.88;

  const scaleMin = 15;
  const scaleMax = 200;
  const scaleStep = 5;
  var scaleCur = 15;

  const btnGridsLogged = document.createElement("button");
  const gridsLogged = new Set();
  let showGridsLogged = false;

  const btnGridsSeen = document.createElement("button");
  const gridsSeenLogged = new Set();
  const gridsSeenNotLogged = new Set();
  const gridsSeenJustLogged = new Set();
  let showGridsSeen = true;


  function setToolTip(elt, tip) {
    elt.className = "tooltip";
    const tipSpan = document.createElement("span");
    tipSpan.innerText = tip;
    tipSpan.className =  "tooltiptext";
    elt.appendChild(tipSpan);
  }

  function setBtnsStateEnable(b) {
    btnGridsSeen.className = showGridsSeen ? "gm_btn_on" : "gm_btn_off";
    btnGridsSeen.enabled = b;
    btnGridsLogged.className = showGridsLogged ? "gm_btn_on" : "gm_btn_off";
    btnGridsLogged.enabled = b;
  }

  function gmBuildHtml() {
    const eltStyle = document.createElement("style");
    document.head.appendChild(eltStyle);
    eltStyle.textContent = 
      ".gm_btn_on { background-color: white; }" + 
      ".gm_btn_off { background-color: lightgray; }" +
      "";
    containerDiv.style = "overflow: hidden; display: inline-block;";
    canvasDiv.appendChild(onsCanvas);
    canvasDiv.className = "boxed";
    canvasDiv.style = "overflow: hidden; " +
      "width: " + width + "px; height:" + height + "px; ";
    zoomSpan.style = "width: 45px; display: inline-flex;";
    const sliderDiv = document.createElement("div");
    slider.type = "range";
    slider.min = scaleMin;
    slider.max = scaleMax;
    slider.value = scaleCur;
    slider.step = scaleStep;
    slider.id="gridzoom";
    slider.style = "width: 150px;";
    sliderDiv.appendChild(zoomSpan);
    sliderDiv.appendChild(slider);
    btnGridsLogged.innerText = "Logged";
    sliderDiv.appendChild(btnGridsLogged);
    btnGridsSeen.innerText = "Seen";
    sliderDiv.appendChild(btnGridsSeen);
    setBtnsStateEnable(false);
    sliderDiv.appendChild(infoDiv);
    infoDiv.appendChild(infoSpan);
    infoSpan.style = "width: 200px; text-align: center; display: inline-flex;"
    setToolTip(infoDiv, "(longitude,latitude) GridId");
    containerDiv.appendChild(canvasDiv);
    containerDiv.appendChild(sliderDiv);
  }

  // Draws offscreen canvas on on-screen canvas scaled
  function gmDrawScaledCanvas(scale) {
    if (scaleCur == scale)
      console.log("redraw");
    scaleCur = scale;
 
    const scaledWidth = ofsCanvas.width * scale/100;
    const scaledHeight = ofsCanvas.height * scale/100;

    onsCanvas.width = scaledWidth;
    onsCanvas.height = scaledHeight;

    onsCtx.drawImage(ofsCanvas, 0, 0, scaledWidth, scaledHeight);
    slider.value = scaleCur;
    zoomSpan.innerText = scale/100;
  }

  function gmToRobinsonPoint(longitude, latitude) {
    const yOfs = ofsCanvas.height * 1.75/100; // 3.25
    const xOfs = ofsCanvas.width * 3.95/100;  // 3.9
    const xOfs1 = ofsCanvas.width * 3.76/100; // 3.8 3.7    
    let point = projection.forward([longitude, latitude])
    point[0] = (point[0] + kx)/(2*kx) * (ofsCanvas.width- 2*xOfs) + xOfs1;
    point[1] = (1.0-(point[1] + ky)/(2*ky)) * (ofsCanvas.height-2*yOfs) + yOfs;
    return point;
  }

  function gmfromRobinsonPoint(mapX, mapY) {
    const yOfs = ofsCanvas.height * 1.75/100; // 3.25
    const xOfs = ofsCanvas.width * 3.95/100;  // 3.9
    const xOfs1 = ofsCanvas.width * 3.76/100; // 3.8 3.7    
    let robX = (mapX - xOfs1)/(ofsCanvas.width - 2*xOfs)*(2*kx) - kx;
    let robY = (1.0 - (mapY - yOfs)/(ofsCanvas.height-2*yOfs)) * (2*ky) - ky;
    let pos = projection.inverse([robX, robY])
    return pos;
  }

  function gmIsValidGridId(gridId) {
    return (gridId.length == 4 && 
      gridId[0] >= 'A' && gridId[0] <= 'R' &&
      gridId[1] >= 'A' && gridId[1] <= 'R' &&
      gridId[2] >= '0' && gridId[2] <= '9' &&
      gridId[3] >= '0' && gridId[3] <= '9' );
  }

  function gmGridIdToPoint(gridId) {
    let point = [0,0];
    if (gmIsValidGridId(gridId)) {
      point[0] = (gridId.charCodeAt(0)-'A'.charCodeAt(0)) * 10 +(gridId.charCodeAt(2)-'0'.charCodeAt(0));
      point[1] = (gridId.charCodeAt(1)-'A'.charCodeAt(0)) * 10 +(gridId.charCodeAt(3)-'0'.charCodeAt(0));
      }
    return point;
  }
  

  function gmSetPix(x,y) {
    const latitude = -y*180.0/ofsCanvas.height  +90;
    const longitude = x*360.0/ofsCanvas.width -180;
    const point = gmToRobinsonPoint(longitude, latitude);
    ofsCtx.fillStyle = "red";
    ofsCtx.fillRect(point[0]-1, point[1]-1, 3, 3);
  }

  function gmMarkPlace(longitude, latitude, clr) {
    const point = gmToRobinsonPoint(longitude, latitude);
    ofsCtx.fillStyle = clr;
    ofsCtx.fillRect(point[0]-1, point[1]-1, 3, 3);
  }

  function gmSetGridMark(col, row, clr) {
    const f = 300.0; //180.0;
    const sx = Math.round(ofsCanvas.width/f);
    const sy = Math.round(ofsCanvas.height/f);
    const dx = Math.round(ofsCanvas.width/f/2);
    const dy = Math.round(ofsCanvas.height/f/2);	
 
    const longitude = col*2-180+0.0;//x*360.0/ofsCanvas.width -180;
    const latitude = row-90+1.0;//-y*180.0/ofsCanvas.height  +90;

    const point = gmToRobinsonPoint(longitude, latitude);

    ofsCtx.fillStyle = clr;
    ofsCtx.fillRect(point[0], point[1], sy, sy);
  }
  
  function gmShowGridId(gridId, clr) {
    const point = gmGridIdToPoint(gridId);
    gmSetGridMark(point[0], point[1], clr);
  }

  function gmSetGridDot(gridId, clr) {
    const point = gmGridIdToPoint(gridId);
    const longitude = point[0]*2-180+0.0;//x*360.0/ofsCanvas.width -180;
    const latitude = point[1]-90+1.0;//-y*180.0/ofsCanvas.height  +90;

    gmMarkPlace(longitude, latitude, clr);
  }

  var pick_x = 0;
  var pick_y = 0;
  var pick_left = 0;
  var pick_top = 0;

  function gmPick(event) {
    const bounding = canvasDiv.getBoundingClientRect();
    pick_x = Math.round(event.clientX - bounding.left);
    pick_y = Math.round(event.clientY - bounding.top);
    pick_left = canvasDiv.scrollLeft;
    pick_top = canvasDiv.scrollTop;
  }

  function gmMouseMove(event) {
    const bounding = canvasDiv.getBoundingClientRect();
    let x = Math.round(event.clientX - bounding.left);
    let y = Math.round(event.clientY - bounding.top);
    if (event.buttons) {
      let sLeft = (x - pick_x) * -1  + pick_left;
      let sTop = (y - pick_y) * -1 + pick_top;
   
      if (sLeft >= 0 && sLeft < canvasDiv.scrollWidth ) 
        canvasDiv.scrollLeft = sLeft;
      if (sTop >= 0 && sTop < canvasDiv.scrollHeight ) 
        canvasDiv.scrollTop = sTop;
    } else {
      const fScaleCur = scaleCur/100;
      const sTop = canvasDiv.scrollTop / fScaleCur;
      const sLeft = canvasDiv.scrollLeft / fScaleCur;
      const mapX = sLeft + x / fScaleCur;
      const mapY = sTop + y / fScaleCur;
      let pos = gmfromRobinsonPoint(mapX, mapY);
      
      x = pos[0]/2+90;
      y = 90 - pos[1];
      if (x >= 0 && x < 180 && y >= 0 && y < 180) {
        let gridId = "";
        gridId += String.fromCharCode(65 + Math.round(x)/10);
        gridId += String.fromCharCode(65 + 18-Math.round(y)/10);
        gridId += String.fromCharCode(48 + Math.round(x)%10);
        gridId += String.fromCharCode(48 + 9-Math.round(y)%10);  

        infoSpan.textContent = '(' + 
          pos[0].toFixed(4) + ',' + pos[1].toFixed(4) + ') ' + gridId;
      } else {
        infoSpan.textContent = '(' + 
          pos[0].toFixed(4) + ',' + pos[1].toFixed(4) + ')';
      }
    }
  }

  function gmZoomToScale(x, y, scale) {
    const fScaleCur = scaleCur/100;
    let sTop = canvasDiv.scrollTop / fScaleCur;
    let sLeft = canvasDiv.scrollLeft / fScaleCur;
    const mapX = sLeft + x / fScaleCur;
    const mapY = sTop + y / fScaleCur;

    if (scale >= scaleMin && scale <= scaleMax) {
      gmDrawScaledCanvas(scale);

      canvasDiv.scrollTop = mapY * scale/100 - y * scale / scaleCur;
      canvasDiv.scrollLeft = mapX * scale/100 - x * scale / scaleCur;
    }
    else {
      console.log("no scale " +  scale + 
        " min " + scaleMin +  " max " + scaleMax);
    }
  }

  function gmMouseZoom(event) {
    const bounding = canvasDiv.getBoundingClientRect();
    const x = Math.round(event.clientX - bounding.left);
    const y = Math.round(event.clientY - bounding.top);  
    event.preventDefault();
    let scale = scaleCur;
    if (event.deltaY > 0)
      scale -= scaleStep;
    else
      scale += scaleStep;
    gmZoomToScale(x, y, scale);
  }
  
  function gmSliderZoom(event) {
    const bounding = canvasDiv.getBoundingClientRect();
    const w = Math.round(bounding.width);
    const h = Math.round(bounding.height);
    gmZoomToScale(w/2, h/2, slider.value);
  }

  function gmLogXYPos(longitude, latitude) {

    const point = projection.forward([longitude, latitude]);

    const px = point[0];
    const py = point[1];

    console.log(`Robinson-projektion: (${longitude},${latitude}) x = ${px}, y = ${py}`);
  }

  function gmGridIdLogged(gridId) {
    gridsSeenLogged.add(gridId);
    if (showGridsSeen) {
      if (!gridsSeenJustLogged.has(gridId)) {
        gmShowGridId(gridId, "darkgreen");
        gmDelayedRefresh();
      }
    }
  }

  function gmGridIdNotLogged(gridId) {
    gridsSeenNotLogged.add(gridId);
    if (showGridsSeen) {
      gmShowGridId(gridId, "rgb(247, 247, 38)"); //yellow
      gmDelayedRefresh();
    }
  }

  function gmGridIdJustLogged(gridId) {
    gridsSeenJustLogged.add(gridId);
    gridsLogged.add(gridId);
    if (showGridsSeen) {
      gmShowGridId(gridId, "red");
      gmDelayedRefresh();
    }
  }

  function clickGridsLogged() {
    showGridsLogged = !showGridsLogged;
    setBtnsStateEnable(false);
    reloadGridMap();
  }

  function clickGridsSeen() {
    showGridsSeen = !showGridsSeen;
    setBtnsStateEnable(false);
    reloadGridMap();
  }

  function gmMarkSeenGridIds() {
    let setIterator = gridsSeenLogged.entries();
    for (const gridId of setIterator) {
      gmShowGridId(gridId[0], "darkgreen");
    }
    setIterator = gridsSeenNotLogged.entries();
    for (const gridId of setIterator) {
      gmShowGridId(gridId[0], "rgb(250, 250, 38)"); //yellow
    }
    setIterator = gridsSeenJustLogged.entries();
    for (const gridId of setIterator) {
      gmShowGridId(gridId[0], "red");
    }
  }

  function gmMarkLoggedGridIds() {
    const setIterator = gridsLogged.entries();
    for (const gridId of setIterator) {
      gmShowGridId(gridId[0], "green");
    }
  }

  let refreshPending = false;
  
  function gmRefresh() {
    refreshPending = false;
    gmDrawScaledCanvas(scaleCur);
  }

  function gmDelayedRefresh() {
    if (!refreshPending) {
      refreshPending = true;
      setTimeout(gmRefresh, 1.0 * 1000);
    }
  }

  function reloadGridMap() {
    ofsCanvas.width = img.width; 
    ofsCanvas.height = img.height;
    ofsCtx.drawImage(img, 0, 0);
    img.style.display = "none";
    if (showGridsLogged) {
      gmMarkLoggedGridIds();
    }
    if (showGridsSeen) {
      gmMarkSeenGridIds();
    }
    gmDrawScaledCanvas(scaleCur);
    setBtnsStateEnable(true);
  }

  let gridIdsLoaded = false;
  let worldMapLoaded = false;

  function gmLoadGridIds() {
    // should be direct db access
    const rnd="?x=" + Math.floor(Math.random() * 10000);
    jQuery.get("grids.txt" + rnd, function (data) {
      let gridId = "";
      for (i = 0; i < data.length; i++) {
        gridId += data[i];
        if (gridId.length == 4) {
          gridsLogged.add(gridId);
          gridId = "";
        }
      }
      gridIdsLoaded = true;
      if (worldMapLoaded) {
        reloadGridMap();
      }
    });
  }
  gmLoadGridIds();

  img.addEventListener("load", () => {
    worldMapLoaded = true;
    if (gridIdsLoaded) {
      reloadGridMap();
    }
  });

  slider.addEventListener('change', gmSliderZoom, false);
  btnGridsLogged.addEventListener("click", clickGridsLogged);
  btnGridsSeen.addEventListener("click", clickGridsSeen);
  canvasDiv.addEventListener("wheel", gmMouseZoom, { passive: false });
  canvasDiv.addEventListener("mousemove", (event) => gmMouseMove(event));
  canvasDiv.addEventListener("mousedown", (event) => gmPick(event));
  //canvasDiv.addEventListener("click", (event) => pick(event));

  return {
    // Offentlige funktioner
    init: function (moduleDiv, w = 640, h = 480) {
      width = w;
      height = h;
      containerDiv = moduleDiv;
      gmBuildHtml();
    },
    redraw: function() { gmDrawScaledCanvas(scaleCur) },
    setGridDot: gmSetGridDot,
    showGridId: gmShowGridId,
    markPlace: gmMarkPlace,
    isValidGridId: gmIsValidGridId,
    gridIdLogged: gmGridIdLogged,
    gridIdNotLogged: gmGridIdNotLogged,
    gridIdJustLogged: gmGridIdJustLogged,
  };

})();
