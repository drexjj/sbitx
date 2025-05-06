"use strict";

const GRIDMAP = (function gridmap() {
  // Private variables and functions
  var containerDiv;
  // Offscreen canvas
  let width;
  let height;
  const ofsCanvas = document.createElement('canvas');
  const ofsCtx = ofsCanvas.getContext('2d');
  // Onscreen canvas (fixed size)
  const canvasDiv = document.createElement("div");
  const onsCanvas = document.createElement("canvas");
  const onsCtx = onsCanvas.getContext('2d');
  const fixedWidth = 520; // Fixed canvas width
  const fixedHeight = 360; // Fixed canvas height

  const slider = document.createElement("input");
  const zoomSpan = document.createElement("span");
  const infoDiv = document.createElement("div");
  const infoSpan = document.createElement("span");

  const btnGridsJustLogged = document.createElement("button");

  const projection = proj4('+proj=merc +lat_0=0 +lon_0=0 +ellps=WGS84 +datum=WGS84 +units=m');
  const img = new Image();

  img.crossOrigin = "anonymous";
  img.src = "./Web_maps_Mercator_projection_SW.jpg";

  const kx = projection.forward([180, 0])[0]; // x-coordinate at (180°, 0°) in meters
  const ky = projection.forward([0, 85])[1];  // y-coordinate at (0°, 85°) in meters

  const scaleMin = 25;
  const scaleMax = 200;
  const scaleStep = 5;
  var scaleCur = 25; // Changed to 30% initial size

  const btnGridsLogged = document.createElement("button");
  const gridsLogged = new Set();
  let showGridsLogged = false;

  const btnGridsSeen = document.createElement("button");
  const gridsSeenLogged = new Set();
  const gridsSeenNotLogged = new Set();
  const gridsSeenJustLogged = new Set();
  let showGridsSeen = true;

  var viewOffsetX = 0; // Tracks panning offset (replaces scrollLeft)
  var viewOffsetY = 0; // Tracks panning offset (replaces scrollTop)

  function setToolTip(elt, tip) {
    elt.className = "tooltip";
    const tipSpan = document.createElement("span");
    tipSpan.innerText = tip;
    tipSpan.className = "tooltiptext";
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
      ".boxed { border: 1px solid black; }";
    containerDiv.style = "overflow: hidden; display: inline-block;";
    canvasDiv.appendChild(onsCanvas);
    canvasDiv.className = "boxed";
    canvasDiv.style = `overflow: hidden; width: ${fixedWidth}px; height: ${fixedHeight}px;`;
    onsCanvas.width = fixedWidth;
    onsCanvas.height = fixedHeight;

    zoomSpan.style = "width: 45px; display: inline-flex;";
    const sliderDiv = document.createElement("div");
    slider.type = "range";
    slider.min = scaleMin;
    slider.max = scaleMax;
    slider.value = scaleCur; // Reflects initial scaleCur = 30
    slider.step = scaleStep;
    slider.id = "gridzoom";
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
    infoSpan.style = "width: 200px; text-align: center; display: inline-flex;";
    setToolTip(infoDiv, "(longitude,latitude) GridId");
    containerDiv.appendChild(canvasDiv);
    containerDiv.appendChild(sliderDiv);
  }

  function gmDrawScaledCanvas(scale) {
    if (scaleCur === scale) {
      console.log("redraw");
    }
    scaleCur = scale;

    const fScale = scale / 100;
    const scaledWidth = ofsCanvas.width * fScale;
    const scaledHeight = ofsCanvas.height * fScale;

    // Clear the onscreen canvas
    onsCtx.clearRect(0, 0, fixedWidth, fixedHeight);

    // Calculate source rectangle based on view offset
    const sourceWidth = fixedWidth / fScale;
    const sourceHeight = fixedHeight / fScale;
    let sourceX = viewOffsetX / fScale;
    let sourceY = viewOffsetY / fScale;

    // Clamp source coordinates to stay within map bounds
    const maxSourceX = ofsCanvas.width - sourceWidth;
    const maxSourceY = ofsCanvas.height - sourceHeight;
    sourceX = Math.max(0, Math.min(sourceX, maxSourceX));
    sourceY = Math.max(0, Math.min(sourceY, maxSourceY));

    // Update view offsets to clamped values
    viewOffsetX = sourceX * fScale;
    viewOffsetY = sourceY * fScale;

    // Draw the map, preserving aspect ratio
    onsCtx.drawImage(
      ofsCanvas,
      sourceX, sourceY, sourceWidth, sourceHeight,
      0, 0, fixedWidth, fixedHeight
    );

    slider.value = scaleCur;
    zoomSpan.innerText = (scale / 100).toFixed(2);
  }

  function gmToMercatorPoint(longitude, latitude) {
    const xOfs = 0;
    const yOfs = 0;
    let point = projection.forward([longitude, latitude]);
    point[0] = (point[0] + kx) / (2 * kx) * ofsCanvas.width + xOfs;
    point[1] = (ky - point[1]) / (2 * ky) * ofsCanvas.height + yOfs;
    return point;
  }

  function gmFromMercatorPoint(mapX, mapY) {
    const xOfs = 0;
    const yOfs = 0;
    let mercX = (mapX - xOfs) / ofsCanvas.width * (2 * kx) - kx;
    let mercY = ky - (mapY - yOfs) / ofsCanvas.height * (2 * ky);
    let pos = projection.inverse([mercX, mercY]);
    return pos;
  }

  function gmIsValidGridId(gridId) {
    return (
      gridId.length == 4 &&
      gridId[0] >= 'A' && gridId[0] <= 'R' &&
      gridId[1] >= 'A' && gridId[1] <= 'R' &&
      gridId[2] >= '0' && gridId[2] <= '9' &&
      gridId[3] >= '0' && gridId[3] <= '9'
    );
  }

  function gmGridIdToPoint(gridId) {
    let point = [0, 0];
    if (gmIsValidGridId(gridId)) {
      point[0] = (gridId.charCodeAt(0) - 'A'.charCodeAt(0)) * 10 + (gridId.charCodeAt(2) - '0'.charCodeAt(0));
      point[1] = (gridId.charCodeAt(1) - 'A'.charCodeAt(0)) * 10 + (gridId.charCodeAt(3) - '0'.charCodeAt(0));
    }
    return point;
  }

  function gmSetPix(x, y) {
    const latitude = -y * 180.0 / ofsCanvas.height + 90;
    const longitude = x * 360.0 / ofsCanvas.width - 180;
    const point = gmToMercatorPoint(longitude, latitude);
    ofsCtx.fillStyle = "red";
    ofsCtx.fillRect(point[0] - 1, point[1] - 1, 3, 3);
  }

  function gmMarkPlace(longitude, latitude, clr) {
    if (latitude > 85 || latitude < -85) return;
    const point = gmToMercatorPoint(longitude, latitude);
    ofsCtx.fillStyle = clr;
    ofsCtx.fillRect(point[0] - 1, point[1] - 1, 3, 3);
  }

  // Replace gmSetGridMark:
  function gmSetGridMark(col, row, clr) {
    const f = 150.0;
    const sx = Math.round(ofsCanvas.width / f);
    const sy = Math.round(ofsCanvas.height / f);
    const radius = Math.min(sx, sy) / 2; // Radius for circles

    const longitude = col * 2 - 180 + 0.0;
    const latitude = row - 90 + 1.0;

    const point = gmToMercatorPoint(longitude, latitude);

    ofsCtx.fillStyle = clr;
    ofsCtx.beginPath();
    ofsCtx.arc(point[0] + radius, point[1] + radius, radius, 0, 2 * Math.PI);
    ofsCtx.fill();
  }
  function gmShowGridId(gridId, clr) {
    const point = gmGridIdToPoint(gridId);
    gmSetGridMark(point[0], point[1], clr);
  }

  function gmSetGridDot(gridId, clr) {
    const point = gmGridIdToPoint(gridId);
    const longitude = point[0] * 2 - 180 + 0.0;
    const latitude = point[1] - 90 + 1.0;
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
    pick_left = viewOffsetX;
    pick_top = viewOffsetY;
  }

  function gmMouseMove(event) {
    const bounding = canvasDiv.getBoundingClientRect();
    let x = Math.round(event.clientX - bounding.left);
    let y = Math.round(event.clientY - bounding.top);
    if (event.buttons) {
      const fScale = scaleCur / 100;
      let newOffsetX = pick_left + (x - pick_x) * -1;
      let newOffsetY = pick_top + (y - pick_y) * -1;

      // Clamp offsets to prevent panning beyond map bounds
      const maxOffsetX = ofsCanvas.width * fScale - fixedWidth;
      const maxOffsetY = ofsCanvas.height * fScale - fixedHeight;
      viewOffsetX = Math.max(0, Math.min(newOffsetX, maxOffsetX));
      viewOffsetY = Math.max(0, Math.min(newOffsetY, maxOffsetY));

      gmDrawScaledCanvas(scaleCur); // Redraw to update view
    } else {
      const fScaleCur = scaleCur / 100;
      const mapX = viewOffsetX / fScaleCur + x / fScaleCur;
      const mapY = viewOffsetY / fScaleCur + y / fScaleCur;
      let pos = gmFromMercatorPoint(mapX, mapY);

      x = pos[0] / 2 + 90;
      y = 90 - pos[1];
      if (x >= 0 && x < 180 && y >= 0 && y < 180 && Math.abs(pos[1]) <= 85) {
        let gridId = "";
        gridId += String.fromCharCode(65 + Math.round(x) / 10);
        gridId += String.fromCharCode(65 + 18 - Math.round(y) / 10);
        gridId += String.fromCharCode(48 + Math.round(x) % 10);
        gridId += String.fromCharCode(48 + 9 - Math.round(y) % 10);

        infoSpan.textContent = '(' +
          pos[0].toFixed(4) + ',' + pos[1].toFixed(4) + ') ' + gridId;
      } else {
        infoSpan.textContent = '(' +
          pos[0].toFixed(4) + ',' + pos[1].toFixed(4) + ')';
      }
    }
  }

  function gmZoomToScale(x, y, scale) {
    if (scale < scaleMin || scale > scaleMax) {
      console.log(`no scale ${scale} min ${scaleMin} max ${scaleMax}`);
      return;
    }

    const fScaleCur = scaleCur / 100;
    const mapX = viewOffsetX / fScaleCur + x / fScaleCur;
    const mapY = viewOffsetY / fScaleCur + y / fScaleCur;

    scaleCur = scale; // Update scale before drawing
    const newScale = scale / 100;

    // Adjust view offset to keep the same map point under the mouse
    viewOffsetX = mapX * newScale - x;
    viewOffsetY = mapY * newScale - y;

    // Clamp offsets to prevent panning beyond map bounds
    const maxOffsetX = ofsCanvas.width * newScale - fixedWidth;
    const maxOffsetY = ofsCanvas.height * newScale - fixedHeight;
    viewOffsetX = Math.max(0, Math.min(viewOffsetX, maxOffsetX));
    viewOffsetY = Math.max(0, Math.min(viewOffsetY, maxOffsetY));

    gmDrawScaledCanvas(scale);
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
    gmZoomToScale(fixedWidth / 2, fixedHeight / 2, parseInt(slider.value));
  }

  function gmLogXYPos(longitude, latitude) {
    const point = projection.forward([longitude, latitude]);
    console.log(`Mercator-projection: (${longitude},${latitude}) x = ${point[0]}, y = ${point[1]}`);
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
      gmShowGridId(gridId, "rgb(247, 247, 38)");
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
      gmShowGridId(gridId[0], "rgb(250, 250, 38)");
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
    console.log(`Map dimensions: ${img.width}x${img.height}`); // Debug

    // Center map horizontally at 30% scale
    const fScale = scaleCur / 100; 
    const scaledWidth = ofsCanvas.width * fScale;
    viewOffsetX = (scaledWidth - fixedWidth) / 2; // Center horizontally
    viewOffsetY = 0; // Top-aligned, as in original

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
    const rnd = "?x=" + Math.floor(Math.random() * 10000);
    jQuery.get("grids.txt" + rnd, function (data) {
      let gridId = "";
      for (let i = 0; i < data.length; i++) {
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

  return {
    init: function (moduleDiv, w = 520, h = 360) {
      width = w;
      height = h;
      containerDiv = moduleDiv;
      gmBuildHtml();
    },
    redraw: function () { gmDrawScaledCanvas(scaleCur); },
    setGridDot: gmSetGridDot,
    showGridId: gmShowGridId,
    markPlace: gmMarkPlace,
    isValidGridId: gmIsValidGridId,
    gridIdLogged: gmGridIdLogged,
    gridIdNotLogged: gmGridIdNotLogged,
    gridIdJustLogged: gmGridIdJustLogged,
  };
})();