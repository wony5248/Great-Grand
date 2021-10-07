import * as React from 'react';
import Box from '@mui/material/Box';
import Grid from '@mui/material/Grid';
import Link from '@mui/material/Link';
import Container from '@mui/material/Container';
import Typography from '../components/Typography';
import TextField from '../components/TextField';
import GitHubIcon from '@mui/icons-material/GitHub';
import Icon from '@mui/material/Icon';
function Copyright() {
  return (
    <React.Fragment>
      {'© '}
      <Link color="inherit" href="https://lab.ssafy.com/s05-iot-ctrl/S05P21A103">
        Our Git
      </Link>{' '}
      {new Date().getFullYear()}
    </React.Fragment>
  );
}

const iconStyle = {
  width: 48,
  height: 48,
  display: 'flex',
  justifyContent: 'center',
  alignItems: 'center',
  backgroundColor: 'white',
  mr: 1,
  cursor: "pointer"
};

const LANGUAGES = [
  {
    code: 'en-US',
    name: 'English',
  },
  {
    code: 'ko-KR',
    name: '한국어',
  },
];

export default function AppFooter() {
  return (
    <Typography
      component="footer"
      sx={{ display: 'flex', bgcolor: 'main.aa' }}
    >
      <Container sx={{ my: 8, display: 'flex' }}>
        <Grid container spacing={5}>
          <Grid item xs={6} sm={4} md={3}>
            <Grid
              container
              direction="column"
              justifyContent="flex-end"
              spacing={2}
              sx={{ height: 120 }}
            >
              <Grid item sx={{ display: 'flex' }}>
                <Box href="https://lab.ssafy.com/s05-iot-ctrl/S05P21A103.git" sx={iconStyle}>
                  <Icon><GitHubIcon></GitHubIcon></Icon>
                </Box>
              </Grid>
              <Grid item>
                <Copyright />
              </Grid>
            </Grid>
          </Grid>
          <Grid item xs={6} sm={4} md={2}>
            <Typography variant="h6" marked="left" gutterBottom>
              Our Team
            </Typography>
            <Box component="ul" sx={{ m: 0, listStyle: 'none', p: 0 }}>
              <Box component="li" sx={{ py: 0.5 }}>
                <Link href="https://lab.ssafy.com/s05-iot-ctrl/S05P21A103.git">신은지</Link>
              </Box>
              <Box component="li" sx={{ py: 0.5 }}>
                <Link href="https://lab.ssafy.com/s05-iot-ctrl/S05P21A103.git">윤익선</Link>
              </Box>
              <Box component="li" sx={{ py: 0.5 }}>
                <Link href="https://lab.ssafy.com/s05-iot-ctrl/S05P21A103.git">이지훈</Link>
              </Box>
              <Box component="li" sx={{ py: 0.5 }}>
                <Link href="https://lab.ssafy.com/s05-iot-ctrl/S05P21A103.git">이희재</Link>
              </Box>
              <Box component="li" sx={{ py: 0.5 }}>
                <Link href="https://lab.ssafy.com/s05-iot-ctrl/S05P21A103.git">장범진</Link>
              </Box>
            </Box>
          </Grid>
          <Grid item xs={6} sm={8} md={4}>
            <Typography variant="h6" marked="left" gutterBottom>
              Language
            </Typography>
            <TextField
              select
              size="medium"
              variant="standard"
              SelectProps={{
                native: true,
              }}
              sx={{ mt: 1, width: 150 }}
            >
              {LANGUAGES.map((language) => (
                <option value={language.code} key={language.code}>
                  {language.name}
                </option>
              ))}
            </TextField>
          </Grid>
          <Grid item>
            <Typography variant="caption">
              {'This site made by '}
              <Link href="https://lab.ssafy.com/s05-iot-ctrl/S05P21A103" rel="sponsored" title="GG">
                Great Grand
              </Link>
              {' is licensed by '}
              <Link
                href="https://creativecommons.org/licenses/by/3.0/"
                title="Creative Commons BY 3.0"
                target="_blank"
                rel="noopener noreferrer"
              >
                CC 3.0 BY
              </Link>
            </Typography>
          </Grid>
        </Grid>
      </Container>
    </Typography>
  );
}
