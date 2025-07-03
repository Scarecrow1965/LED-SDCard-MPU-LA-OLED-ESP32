// All from https://randomnerdtutorials.com/esp32-mpu-6050-web-server/
/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp32-mpu-6050-web-server/

  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*/

let scene, camera, rendered, cube, renderer;

function parentWidth(elem) {
  return elem.parentElement.clientWidth;
};

function parentHeight(elem) {
  return elem.parentElement.clientHeight;
};

function init3D() {
  scene = new THREE.Scene();
  scene.background = new THREE.Color(0xffffff);

  camera = new THREE.PerspectiveCamera(
    75,
    parentWidth(document.getElementById("3Dcube")) /
      parentHeight(document.getElementById("3Dcube")),
    0.1,
    1000
  );

  renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setSize(
    parentWidth(document.getElementById("3Dcube")),
    parentHeight(document.getElementById("3Dcube"))
  );

  document.getElementById("3Dcube").appendChild(renderer.domElement);

  // Create a geometry
  const geometry = new THREE.BoxGeometry(5, 1, 4);

  // Materials of each face
  var cubeMaterials = [
    new THREE.MeshBasicMaterial({ color: 0x03045e }),
    new THREE.MeshBasicMaterial({ color: 0x023e8a }),
    new THREE.MeshBasicMaterial({ color: 0x0077b6 }),
    new THREE.MeshBasicMaterial({ color: 0x03045e }),
    new THREE.MeshBasicMaterial({ color: 0x023e8a }),
    new THREE.MeshBasicMaterial({ color: 0x0077b6 }),
  ];

  const material = new THREE.MeshFaceMaterial(cubeMaterials);

  cube = new THREE.Mesh(geometry, material);
  scene.add(cube);
  camera.position.z = 5;
  renderer.render(scene, camera);
}; // end of init3D function

// Resize the 3D object when the browser window changes size
function onWindowResize() {
  // Explicitly target the 3D cube container
  const cubeContainer = document.getElementById("3Dcube");
  if (cubeContainer) {
    const width = parentWidth(cubeContainer);
    const height = parentHeight(cubeContainer);

    // Update camera aspect based on the 3D cube container's dimensions
    camera.aspect = width / height;
    camera.updateProjectionMatrix();

    // Update renderer size to match the 3D cube container
    renderer.setSize(width, height);
  }
  
  // original code
  // camera.aspect =
  //   parentWidth(document.getElementById("3Dcube")) /
  //   parentHeight(document.getElementById("3Dcube"));
  // // camera.aspect = window.innerWidth /  window.innerHeight;
  // camera.updateProjectionMatrix();
  // // renderer.setSize(window.innerWidth, window.innerHeight);
  // renderer.setSize(
  //   parentWidth(document.getElementById("3Dcube")),
  //   parentHeight(document.getElementById("3Dcube"))
  // );
}; // end of onWindowResize function

window.addEventListener("resize", onWindowResize, false);

// Create the 3D representation
init3D();

// Create events for the sensor readings
if (!!window.EventSource) {
  var source = new EventSource("/events");

  source.addEventListener(
    "open",
    function (e) {
      console.log("Events Connected");
    },
    false
  );

  source.addEventListener(
    "error",
    function (e) {
      if (e.target.readyState != EventSource.OPEN) {
        console.log("Events Disconnected");
      }
    },
    false
  );

  source.addEventListener(
    "MPU_readings",
    function (e) {
      // used for testing purposes only
      // console.log("gyro_readings", e.data);
      // 
      var obj = JSON.parse(e.data);
      document.getElementById("gyroX").innerHTML = obj.gyroX;
      document.getElementById("gyroY").innerHTML = obj.gyroY;
      document.getElementById("gyroZ").innerHTML = obj.gyroZ;

      document.getElementById("accX").innerHTML = obj.accX;
      document.getElementById("accY").innerHTML = obj.accY;
      document.getElementById("accZ").innerHTML = obj.accZ;

      console.log("temperature_reading", e.data);
      document.getElementById("temp").innerHTML = e.data;

      // Change cube rotation after receiving the readings
      cube.rotation.x = obj.gyroY;
      cube.rotation.z = obj.gyroX;
      cube.rotation.y = obj.gyroZ;

      renderer.render(scene, camera);
    },
    false
  );

  // original code
  // source.addEventListener(
  //   "temperature_reading",
  //   function (e) {
  //     console.log("temperature_reading", e.data);
  //     document.getElementById("temp").innerHTML = e.data;
  //   },
  //   false
  // );

  // original code
  // source.addEventListener(
  //   "accelerometer_readings",
  //   function (e) {
  //     console.log("accelerometer_readings", e.data);
  //     var obj = JSON.parse(e.data);
  //     document.getElementById("accX").innerHTML = obj.accX;
  //     document.getElementById("accY").innerHTML = obj.accY;
  //     document.getElementById("accZ").innerHTML = obj.accZ;
  //   },
  //   false
  // );
}; // end of if statement

function resetPosition(element) {
  var xhr = new XMLHttpRequest();
  xhr.open("GET", "/" + element.id, true);
  console.log(element.id);
  xhr.send();
}; // end of resetPosition function

// ========================
// TESTING AREA ONLY
// DIGITAL DISPLAY
// ========================

// this class is built for retrieveinng and displaying the height of the table using the RT-11
class DigitalNumber extends HTMLElement {
  constructor() {
    super();
    this._height = 0; // default height
  }

  updateElement() {
    // Clear existing content or adjust as necessary
    this.innerHTML = ""; // Example: Clear existing content to update. Adjust as needed.

    // Assuming this._height is the variable you want to check
    if (this.height <= 9) {
      // If height is 9 or below, use digital-digit
      // const digitElement = document.createElement("digital-digit");
      const digitElement = document.querySelector<DigitalDigitElement>("display-digit");
      // Set properties or attributes of digitElement as needed
      digitElement.setAttribute("display-digit", this.height.toString());
      // Append digitElement to the DOM
      document.body.appendChild(displayDigit); // Assuming you want to append to the body
    } else if (this.height >= 10) {
      // If height is 10 or above, use digital-number
      const numberElement = document.createElement("display-number");
      // Set properties or attributes of numberElement as needed
      numberElement.setAttribute("display-number", this.height.toString());
      // Append numberElement to the DOM
      document.body.appendChild(numberElement); // Assuming you want to append to the body
    }
  };

// Function to update the height in the HTML
updateHeight() {
    // Select the digital-digit and digital-number elements
    const digitalDigitElement = document.querySelector('digital-digit');
    const digitalNumberElement = document.querySelector('digital-number');
  
    // Set the display-digit or display-number attribute to the height
    if (digitalDigitElement) digitalDigitElement.setAttribute('display-digit', height);
    if (digitalNumberElement) digitalNumberElement.setAttribute('display-number', height);
  }
  
  // Getter for height
  get height() {
    return this._height;
  }

  // Setter for height
  set height(newHeight) {
    newHeight = parseInt(newHeight);
    if (!isNaN(newHeight)) {
      this._height = newHeight;
      this.style.height = `${newHeight}px`; // Update the element's height
    }
  }
  /**
   * Let the browser know what attributes to keep track of
   */
  static get observedAttributes() {
    return ["height"];
  }

  /**
   * When the user changes the attribute, this will be invoked
   */
  attributeChangedCallback(name, oldValue, newValue) {
    switch (name) {
      case "height":
        this.height = newValue;
        this.updateElement(); // Call a method to update the element based on the new height
        break;
    }
  }

}; // end of DigitalNumber class

customElements.define("DigitalNumber", DigitalNumber);

// The array is made up of 1's to light up the cells in the LED display, the 0.1's leave them virtually (opacity) off.
// If you look at a digit, it's made up of 7 possible "lights" - top (a), topleft (b), top right (c), middle (d), bottom left (e), bottom right (f) and bottom (g).
// The first 7 digits in the array make the number zero.
// The second 7 digits make the number 1, and so on... 0 to 9, in clusters of 7 bits for each digit
// 0 = off, 1 = on
// The decimal point is the 8th bit, but I'm not using it in this example.
// BUT IAW https://en.m.wikipedia.org/wiki/Seven-segment_display
// the individual segments are labelled a to g, with the decimal point as h.
// segment a = top, b = top right, c = bottom right, d = bottom, e = bottom left, f = top left, g = middle, h = decimal point
// therefore the numbering system is: a, b, c, d, e, f, g, h
// BUT IAW https://en.m.wikipedia.org/wiki/Fourteen-segment_display
// the individual segments are labelled a, b, c, d, e, f, g1, g2, h, i, j,  k , l, m, with the decimal point as dp.
// segments a = top, b = top right, c = bottom right, d = bottom, e = bottom left, f = top left, g1 (left) and g2 (right) are the middle segments,
// h = top middle slant left_side, i = top middle middle, j = top middle slant right_side, k = bottom middle slant left_side, l = bottom middle middle,
// m = bottom middle slant left_side, dp = decimal point

// from: https://bytethisstore.com/articles/pg/js-digital-numbers

// ========================-========================
// from digital-digit.js
// ========================-========================
/**
 * Element which holds a single digital digit
 */
class DigitalDigitElement extends HTMLElement {
  constructor() {
    super();
    this._displayDigit = 8; // number of digit to display on screen -1 = 7
  }

  /**
   * We'll use getters and setters so
   *  we can control what happens when updated
   */
  get displayDigit() {
    return this._displayDigit;
  }

  set displayDigit(newDigit) {
    newDigit = parseInt(newDigit);
    if (isNaN(newDigit)){
      newDigit = 0;
    }
    this._displayDigit = newDigit;
    this._updateChildren();
  }

  /**
   * Let the browser know what attributes to keep track of
   */
  static get observedAttributes() {
    return ["display-digit"];
  }

  /**
   * When the user changes the attribute, this will be invoked
   */
  attributeChangedCallback(name, oldValue, newValue) {
    switch (name) {
      case "display-digit":
        this.displayDigit = newValue;
        break;
    }
  }

  /**
   * This is fired by the browser when the element
   *  is placed onto the DOM
   *
   * Initialize the content children here
   */
  connectedCallback() {
    /**
     * Display will look (something) like
     *
     *  _
     * | |
     *  _
     * | |
     *  _
     */
    // also known as a 7 digit display

    // this property will help position the child elements
    this.style.position = "relative";

    //create the individual clock "pieces"
    this._topHorz = document.createElement("div");
    this._midHorz = document.createElement("div");
    this._bottomHorz = document.createElement("div");

    this._topLeftVert = document.createElement("div");
    this._topRightVert = document.createElement("div");
    this._bottomLeftVert = document.createElement("div");
    this._bottomRightVert = document.createElement("div");

    //store all in array for convenience
    this._allClockEl = [
      this._topHorz,
      this._midHorz,
      this._bottomHorz,
      this._topLeftVert,
      this._topRightVert,
      this._bottomLeftVert,
      this._bottomRightVert,
    ];

    //add the styles to position horizontal elements
    [this._topHorz, this._midHorz, this._bottomHorz].forEach((horz) => {
      horz.style.width = "85%";
      horz.style.height = "4%";

      horz.style.position = "absolute";
      horz.style.left = "7.5%";

      this.appendChild(horz);
    });
    this._midHorz.style.top = "48%";
    this._bottomHorz.style.top = "95%";

    //add styles to position vertical elements
    [
      this._topLeftVert,
      this._topRightVert,
      this._bottomLeftVert,
      this._bottomRightVert,
    ].forEach((vert) => {
      vert.style.width = "8%";
      vert.style.height = "40%";

      vert.style.position = "absolute";

      this.appendChild(vert);
    });

    //additional positioning
    [this._topLeftVert, this._topRightVert].forEach((topVert) => {
      topVert.style.top = "6%";
    });
    [this._bottomLeftVert, this._bottomRightVert].forEach((bottomVert) => {
      bottomVert.style.top = "53%";
    });
    [this._topRightVert, this._bottomRightVert].forEach((rightVert) => {
      rightVert.style.left = "92%";
    });

    this._updateChildren();
  } // end of connectedCallback

  /**
   * Update the chilren elements to activate bits
   */
  _updateChildren() {
    if (!this._allClockEl) {
      return;
    }

    //refresh active states of children
    this._allClockEl.forEach((el) => {
      //control active states using CSS classes
      el.classList.remove("digital-digit-active");
      el.classList.add("digital-digit-inactive");
    });

    let activated = [];
    //update the class names
    switch (this.displayDigit) {
      case 0:
        activated = [
          this._topHorz,
          this._bottomHorz,
          this._topLeftVert,
          this._topRightVert,
          this._bottomLeftVert,
          this._bottomRightVert,
        ];
        break;
      case 1:
        activated = [this._topRightVert, this._bottomRightVert];
        break;
      case 2:
        activated = [
          this._topHorz,
          this._midHorz,
          this._bottomHorz,
          this._topRightVert,
          this._bottomLeftVert,
        ];
        break;
      case 3:
        activated = [
          this._topHorz,
          this._bottomHorz,
          this._midHorz,
          this._topRightVert,
          this._bottomRightVert,
        ];
        break;
      case 4:
        activated = [
          this._topLeftVert,
          this._topRightVert,
          this._midHorz,
          this._bottomRightVert,
        ];
        break;
      case 5:
        activated = [
          this._topHorz,
          this._topLeftVert,
          this._midHorz,
          this._bottomRightVert,
          this._bottomHorz,
        ];
        break;
      case 6:
        activated = [
          this._topHorz,
          this._midHorz,
          this._bottomHorz,
          this._topLeftVert,
          this._bottomLeftVert,
          this._bottomRightVert,
        ];
        break;
      case 7:
        activated = [this._topHorz, this._topRightVert, this._bottomRightVert];
        break;
      case 8:
        activated = this._allClockEl;
        break;
      case 9:
        activated = [
          this._topHorz,
          this._midHorz,
          this._topLeftVert,
          this._topRightVert,
          this._bottomRightVert,
        ];
        break;
    }

    activated.forEach((el) => {
      el.classList.add("digital-digit-active");
      el.classList.remove("digital-digit-inactive");
    });
  } // end of _updateChildren

}; // end of DigitalDigitElement class

//add to custom elements registry
customElements.define("digital-digit", DigitalDigitElement);


// ========================-========================
// from digital-number.js
// ========================-========================
/**
 * Element which holds a multiple digit number
 */
class DigitalNumberElement extends HTMLElement {
  constructor() {
    super();
    this._displayNumber = 12; // default number to display
  }

  /**
   * We'll use getters and setters so
   *  we can control what happens when updated
   */
  get displayNumber() {
    return this._displayNumber;
  }

  set displayNumber(newNumber) {
    newNumber = parseInt(newNumber);
    if (isNaN(newNumber)) {
      newNumber = 0;
    }
    this._displayNumber = newNumber;

    this._updateChildren();
  }

  /**
   * Let the browser know what attributes to keep track of
   */
  static get observedAttributes() {
    return ["display-number"];
  }

  /**
   * When the user changes the attribute, this will be invoked
   */
  attributeChangedCallback(name, oldValue, newValue) {
    switch (name) {
      case "display-number":
        this.displayNumber = newValue;
        break;
    }
  }

  connectedCallback() {
    this._updateChildren();
  }

  _updateChildren() {
    //clear self
    this.innerHTML = "";

    //add one digit for each digit in number
    let n = this._displayNumber;
    while (n >= 10) {
      const digit = n % 10;
      //create an instance of our other web component
      const digitElement = document.createElement("digital-digit");
      digitElement.displayDigit = digit;
      this.insertBefore(digitElement, this.firstChild);

      n = Math.floor(n / 10);
    }

    //append remaining digit
    const digitElement = document.createElement("digital-digit");
    digitElement.displayDigit = n;
    this.insertBefore(digitElement, this.firstChild);
  }
}; // end of DigitalNumberElement class

//add to registry
customElements.define("digital-number", DigitalNumberElement);
