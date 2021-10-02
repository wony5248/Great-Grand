import React, { useEffect } from "react";
import { BrowserRouter, Switch, Route, Redirect } from "react-router-dom";
import SignIn from "./pages/SignIn";
import SignUp from "./pages/SignUp";
import Home from "./pages/Home";
import ForgotPassword from "./pages/ForgotPassword";
import Privacy from "./pages/Privacy";
import axios from "axios";
import Terms from "./pages/Terms";
import moment from "moment";
import { useState } from "react";

import './App.css';
import Control from "./pages/Control";

function App() {
  return (
    <BrowserRouter>
        <Switch>
          <Route exact path="/" component={Home} />
          <Route exact path="/SignIn" component={SignIn} />
          <Route exact path="/SignUp" component={SignUp} />
          <Route exact path="/ForgotPassword" component={ForgotPassword} />
          <Route exact path="/Privacy" component={Privacy} />
          <Route exact path="/Term" component={Terms} />
          <Route exact path="/Control" component={Control} />
          <Redirect to="/" />
        </Switch>
      </BrowserRouter>

  );
}

export default App;
