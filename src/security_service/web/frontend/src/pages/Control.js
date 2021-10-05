import * as React from "react";
import Container from "@mui/material/Container";
import Box from "@mui/material/Box";
import Markdown from "../components/Markdown";
import Typography from "@mui/material/Typography";
import AppAppBar from "../views/AppAppBar";
import AppFooter from "../views/AppFooter";
import withRoot from "../modules/withRoot";
import terms from "../views/terms.md";
import styled1 from "styled-components";
import { Grid, Paper } from "@mui/material";
import { styled } from "@mui/material/styles";
import { useMediaQuery } from "react-responsive";
import { useEffect, useState } from "react";
// import Streamimg from "../assets/cam.jpg";
import { display } from "@mui/system";
import { io } from "socket.io-client";
const address = "http://127.0.0.1:12001";
// import socketio from "socke"
const Streamingdiv = styled1.div`
    width: 100%;
    height: 300px ;
`;
const Iotcontroldiv = styled1.div`
    width: 100%;
`;
const Iotonoffldiv = styled1.div`
    width: 100%;
    // height: 100px ;
    background-color:#E6F5FF;
`;
const Tuttlediv = styled1.div`
    width: 100%;
`;
const Tuttlecontroldiv = styled1.div`
    width: 100%;
    height: 300px ;
    background-color:#E6F5FF;
`;
const Rescontroldiv = styled1.div`
    width: 100%;
`;
const Onbtn = styled1.button`
    background-color: #00DD88;
    width: 160px;
    height: 80px;
    border:none;
    cursor: pointer;
    color: white;
`;
const Offbtn = styled1.button`
    background-color: #2B2B2B;
    width: 160px;
    height: 80px;
    border: none;
    cursor: pointer;
    color: white;
`;
const Controlbtn = styled1.button`
    background-color: #9AC0E3;
    width: 90px;
    height: 90px;
    border:none;
    cursor: pointer;
    color: white;
`;
const Registbtn = styled1.button`
    background-color: #9AC0E3;
    width: 160px;
    height: 80px;
    border:none;
    cursor: pointer;
    color: white;
`;
const Desktop = ({ children }) => {
  const isDesktop = useMediaQuery({ minWidth: 1061 });
  return isDesktop ? children : null;
};
const Tablet = ({ children }) => {
  const isTablet = useMediaQuery({ minWidth: 613, maxWidth: 1060 });
  return isTablet ? children : null;
};
const Mobile = ({ children }) => {
  const isMobile = useMediaQuery({ maxWidth: 612 });
  return isMobile ? children : null;
};
const Click = (con) => {
  const socket = io(address, {
    withCredentials: true,
    transports: ["websocket", "polling"],
    extraHeaders: {
      "my-custom-header": "abcd",
    },
  });
  socket.emit("IoT", con);
  console.log(con)
};

function Control() {
  const [image, setImage] = useState("");
  useEffect(() => {
    const socket = io(address, {
      withCredentials: true,
      transports: ["websocket", "polling"],
      extraHeaders: {
        "my-custom-header": "abcd",
      },
    });
    socket.on("connect", () => {
      console.log(socket.id);
    });
    socket.on("jpgstream", (msg) => {
      setImage(msg);
    });
    socket.on("disconnect", () => {
      console.log("disconnected");
    });
    console.log(socket.connected);
    socket.on("sensormsg", (message) => {});
  }, []);
  return (
    <React.Fragment>
      <AppAppBar />
      <Container>
        <Box
          sx={{ mt: 7, mb: 12 }}
          style={{
            display: "flex",
            flexDirection: "column",
            alignItems: "center",
          }}
        >
          <Desktop>
            <Streamingdiv style={{ height: "450px", maxWidth: "900px" }}>
              <img
                src={`data:image/jpg;base64,${image}`}
                height="100%"
                width="100%"
              ></img>
            </Streamingdiv>
            <Rescontroldiv style={{ maxWidth: "900px", display: "flex" }}>
              <Iotcontroldiv>
                {" "}
                <Typography
                  style={{
                    marginBottom: "12px",
                    marginTop: "12px",
                    fontWeight: "bold",
                    fontSize: "24px",
                    color: "#9AC0E3",
                  }}
                >
                  IOT ON/OFF
                </Typography>
                <Iotonoffldiv
                  style={{
                    display: "flex",
                    width: "60%",
                    height: "300px",
                    flexDirection: "column",
                    justifyContent: "space-evenly",
                    alignItems: "center",
                  }}
                >
                  <Onbtn onClick={() => Click(1)}>ON</Onbtn>
                  <Offbtn onClick={() => Click(2)}>OFF</Offbtn>
                </Iotonoffldiv>
              </Iotcontroldiv>
              <Tuttlediv>
                <Typography
                  style={{
                    marginBottom: "12px",
                    marginTop: "12px",
                    fontWeight: "bold",
                    fontSize: "24px",
                    color: "#9AC0E3",
                  }}
                >
                  TUTTLE CONTROL
                </Typography>
                <Tuttlecontroldiv>
                  <div style={{ display: "flex", justifyContent: "center" }}>
                    <Controlbtn style={{ marginTop: "15px" }}>GO</Controlbtn>
                  </div>
                  <div
                    style={{ display: "flex", justifyContent: "space-around" }}
                  >
                    <Controlbtn>LEFT</Controlbtn>
                    <Controlbtn>RIGHT</Controlbtn>
                  </div>
                  <div style={{ display: "flex", justifyContent: "center" }}>
                    <Controlbtn>BACK</Controlbtn>
                  </div>
                </Tuttlecontroldiv>
              </Tuttlediv>
            </Rescontroldiv>
            <div
              style={{
                width: "100%",
                maxWidth: "900px",
                marginTop: "30px",
                display: "flex",
                justifyContent: "flex-end",
              }}
            >
              <Registbtn>IOT 등록</Registbtn>
            </div>
          </Desktop>
          <Tablet>
            <Streamingdiv style={{ maxWidth: "700px" }}>
              <img
                src={`data:image/jpg;base64,${image}`}
                height="100%"
                width="100%"
              ></img>
            </Streamingdiv>
            <Rescontroldiv style={{ maxWidth: "700px", display: "flex" }}>
              <Iotcontroldiv>
                {" "}
                <Typography
                  style={{
                    marginBottom: "12px",
                    marginTop: "12px",
                    fontWeight: "bold",
                    fontSize: "24px",
                    color: "#9AC0E3",
                  }}
                >
                  IOT ON/OFF
                </Typography>
                <Iotonoffldiv
                  style={{
                    display: "flex",
                    width: "60%",
                    height: "300px",
                    flexDirection: "column",
                    justifyContent: "space-evenly",
                    alignItems: "center",
                  }}
                >
                  <Onbtn onClick={Click(1)}>ON</Onbtn>
                  <Offbtn onClick={Click(2)}>OFF</Offbtn>
                </Iotonoffldiv>
              </Iotcontroldiv>
              <Tuttlediv>
                <Typography
                  style={{
                    marginBottom: "12px",
                    marginTop: "12px",
                    fontWeight: "bold",
                    fontSize: "24px",
                    color: "#9AC0E3",
                  }}
                >
                  TUTTLE CONTROL
                </Typography>
                <Tuttlecontroldiv>
                  <div style={{ display: "flex", justifyContent: "center" }}>
                    <Controlbtn style={{ marginTop: "15px" }}>GO</Controlbtn>
                  </div>
                  <div
                    style={{ display: "flex", justifyContent: "space-around" }}
                  >
                    <Controlbtn>LEFT</Controlbtn>
                    <Controlbtn>RIGHT</Controlbtn>
                  </div>
                  <div style={{ display: "flex", justifyContent: "center" }}>
                    <Controlbtn>BACK</Controlbtn>
                  </div>
                </Tuttlecontroldiv>
              </Tuttlediv>
            </Rescontroldiv>
            <div
              style={{
                width: "100%",
                maxWidth: "700px",
                marginTop: "30px",
                display: "flex",
                justifyContent: "flex-end",
              }}
            >
              <Registbtn>IOT 등록</Registbtn>
            </div>
          </Tablet>
          <Mobile>
            <Streamingdiv style={{ height: "300px" }}>
              <img
                src={`data:image/jpg;base64,${image}`}
                height="100%"
                width="100%"
              ></img>
            </Streamingdiv>
            <Rescontroldiv>
              <Iotcontroldiv>
                {" "}
                <Typography
                  style={{
                    marginBottom: "12px",
                    marginTop: "12px",
                    fontWeight: "bold",
                    fontSize: "24px",
                    color: "#9AC0E3",
                  }}
                >
                  IOT ON/OFF
                </Typography>
                <Iotonoffldiv
                  style={{
                    display: "flex",
                    justifyContent: "space-around",
                    alignItems: "center",
                  }}
                >
                  <Onbtn onClick={Click(1)}>ON</Onbtn>
                  <Offbtn onClick={Click(2)}>OFF</Offbtn>
                </Iotonoffldiv>
              </Iotcontroldiv>
              <Tuttlediv>
                <Typography
                  style={{
                    marginBottom: "12px",
                    marginTop: "12px",
                    fontWeight: "bold",
                    fontSize: "24px",
                    color: "#9AC0E3",
                  }}
                >
                  TUTTLE CONTROL
                </Typography>
                <Tuttlecontroldiv>
                  <div style={{ display: "flex", justifyContent: "center" }}>
                    <Controlbtn style={{ marginTop: "15px" }}>GO</Controlbtn>
                  </div>
                  <div
                    style={{ display: "flex", justifyContent: "space-around" }}
                  >
                    <Controlbtn>LEFT</Controlbtn>
                    <Controlbtn>RIGHT</Controlbtn>
                  </div>
                  <div style={{ display: "flex", justifyContent: "center" }}>
                    <Controlbtn>BACK</Controlbtn>
                  </div>
                </Tuttlecontroldiv>
              </Tuttlediv>
            </Rescontroldiv>
            <div
              style={{
                width: "100%",
                marginTop: "30px",
                display: "flex",
                justifyContent: "flex-end",
              }}
            >
              <Registbtn>IOT 등록</Registbtn>
            </div>
          </Mobile>
        </Box>
      </Container>
      <AppFooter />
    </React.Fragment>
  );
}

export default withRoot(Control);
