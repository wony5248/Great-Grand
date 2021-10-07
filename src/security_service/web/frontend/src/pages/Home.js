import * as React from 'react';
import AppFooter from '../views/AppFooter';
import ProductHero from '../views/ProductHero';
import ProductValues from '../views/ProductValues';
import ProductHowItWorks from '../views/ProductHowItWorks';
import AppAppBar from '../views/AppAppBar';
import withRoot from '../modules/withRoot';
import ProductIntro from "../views/ProductIntro"

function Index() {
  return (
    <React.Fragment>
      <AppAppBar />
      <ProductHero />
      <ProductIntro/>
      <ProductValues />
      {/* <ProductHowItWorks /> */}
      <AppFooter />
    </React.Fragment>
  );
}

export default withRoot(Index);
