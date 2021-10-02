import * as React from 'react';
import Box from '@mui/material/Box';
import Grid from '@mui/material/Grid';
import Container from '@mui/material/Container';
import Typography from '../components/Typography';
import IoT from "../assets/internet-of-things.svg"
import Robber from "../assets/robber.svg"
import Routine from "../assets/routine.svg"
import TTS from "../assets/text-to-speech.svg"
import Down from "../assets/knock-down.svg"
import Drive from "../assets/steering-wheel.svg"
const item = {
  display: 'flex',
  flexDirection: 'column',
  alignItems: 'center',
  px: 5,
};

function ProductValues() {
  return (
    <Box
      component="section"
      sx={{ display: 'flex', overflow: 'hidden', bgcolor: 'main.background' }}
    >
      <Container sx={{ mt: 15, mb: 30, display: 'flex', position: 'relative' }}>
        <Grid container spacing={5}>
          <Grid item xs={12} md={6} marginTop={12}>
            <Box sx={item}>
              <Box
                component="img"
                src={IoT}
                alt="suitcase"
                sx={{ height: 55 }}
              />
              <Typography variant="h6" sx={{ my: 5 }}>
                IoT 제어
              </Typography>
              <Typography variant="h5">
                {
                  'ROS2를 이용하여 IoT 제어를 할 수 있는 각종 S/W를 제공하여 간편한 조작으로 실시간 IoT 제어를 할 수 있습니다.'
                }
              </Typography>
            </Box>
          </Grid>
          <Grid item xs={12} md={6} marginTop={12}>
            <Box sx={item}>
              <Box
                component="img"
                src={Down}
                alt="suitcase"
                sx={{ height: 55 }}
              />
              <Typography variant="h6" sx={{ my: 5 }}>
                낙상 및 화재 감지
              </Typography>
              <Typography variant="h5">
                {
                  'YOLO v3 모델을 이용한 Object_Detection을 통해 사용자의 낙상 및 화재를 감지하여 알람을 보내주고 보호자에게 연락해 줍니다.'
                }

              </Typography>
            </Box>
          </Grid>
          <Grid item xs={12} md={6} marginTop={12}>
            <Box sx={item}>
              <Box
                component="img"
                src={Drive}
                alt="suitcase"
                sx={{ height: 55 }}
              />
              <Typography variant="h6" sx={{ my: 5 }}>
                자율주행 모니터링
              </Typography>
              <Typography variant="h5">
                {
                  'A* 알고리즘을 이용하여 터틀봇이 최적의 경로를 이용하여 주기적으로 집안을 돌아다녀 모니터링 할 수 있게 해줍니다.'
                }

              </Typography>
            </Box>
          </Grid>
          <Grid item xs={12} md={6} marginTop={12}>
            <Box sx={item}>
              <Box
                component="img"
                src={Routine}
                alt="graph"
                sx={{ height: 55 }}
              />
              <Typography variant="h6" sx={{ my: 5 }}>
                순찰 기능
              </Typography>
              <Typography variant="h5">
                {
                  '원하는 시간에 정해진 경로를 통해 코스를 등록할 수 있고, 터틀봇은 선택되어 있는 코스를 시간에 맞춰 순찰합니다.'
                }
              </Typography>
            </Box>
          </Grid>
          <Grid item xs={12} md={6} marginTop={12}>
            <Box sx={item}>
              <Box
                component="img"
                src={TTS}
                alt="clock"
                sx={{ height: 55 }}
              />
              <Typography variant="h6" sx={{ my: 5 }}>
                음성 인식 및 음성 안내
              </Typography>
              <Typography variant="h5">
                {'STT 와 TTS를 이용하여 음성으로 음성으로 조작이 가능하고 음성으로 알람을 받는것이 가능합니다.'}
              </Typography>
            </Box>
          </Grid>
          <Grid item xs={12} md={6} marginTop={12}>
            <Box sx={item}>
              <Box
                component="img"
                src={Robber}
                alt="clock"
                sx={{ height: 55 }}
              />
              <Typography variant="h6" sx={{ my: 5 }}>
                사용자 인식 및 침입자 감지
              </Typography>
              <Typography variant="h5">
                {'Face_Recognition을 이용하여 사용자를 등록할 수 있고, 등록되지 않은 사용자가 인식 될 시에 알람을 주고 상황에 따라 연락을 보내줍니다.'}
              </Typography>
            </Box>
          </Grid>
        </Grid>
      </Container>
    </Box>
  );
}

export default ProductValues;
