import * as React from "react";
import Box from "@mui/material/Box";
import Grid from "@mui/material/Grid";
import Link from "@mui/material/Link";
import { Field, Form, FormSpy } from "react-final-form";
import Typography from "../components/Typography";
import AppFooter from "../views/AppFooter";
import AppAppBar from "../views/AppAppBar";
import AppForm from "../views/AppForm";
import { email, required } from "../components/form/validation";
import RFTextField from "../components/form/RFTextField";
import FormButton from "../components/form/FormButton";
import FormFeedback from "../components/form/FormFeedback";
import withRoot from "../modules/withRoot";

function SignUp() {
  const [sent, setSent] = React.useState(false);

  const validate = (values) => {
    const errors = required(
      ["passwordcheck", "name", "email", "password", "phonenum"],
      values
    );

    if (!errors.email) {
      const emailError = email(values.email);
      if (emailError) {
        errors.email = emailError;
      }
    }

    return errors;
  };

  const handleSubmit = () => {
    setSent(true);
  };

  return (
    <React.Fragment>
      <AppAppBar />
      <AppForm>
        <React.Fragment>
          <Typography variant="h3" gutterBottom marked="center" align="center">
            Sign Up
          </Typography>
          <Typography variant="body2" align="center">
            <Link href="/SignIn" underline="always">
              Already have an account?
            </Link>
          </Typography>
        </React.Fragment>
        <Form
          onSubmit={handleSubmit}
          subscription={{ submitting: true }}
          validate={validate}
        >
          {({ handleSubmit: handleSubmit2, submitting }) => (
            <Box
              component="form"
              onSubmit={handleSubmit2}
              noValidate
              sx={{ mt: 6 }}
            >
              <Grid container spacing={2}>
                <Grid item xs={12} sm={12}>
                  <Field
                    autoFocus
                    component={RFTextField}
                    disabled={submitting || sent}
                    autoComplete="name"
                    fullWidth
                    label="이름"
                    name="name"
                    required
                  />
                </Grid>
                <Grid item xs={12} sm={12}>
                  <Field
                    autoComplete="email"
                    component={RFTextField}
                    disabled={submitting || sent}
                    fullWidth
                    label="이메일"
                    margin="normal"
                    name="email"
                    required
                  />
                </Grid>
                <Grid item xs={12} sm={6}>
                  <Field
                    fullWidth
                    component={RFTextField}
                    disabled={submitting || sent}
                    required
                    name="password"
                    autoComplete="new-password"
                    label="비밀번호"
                    type="password"
                    margin="normal"
                  />
                </Grid>
                <Grid item xs={12} sm={6}>
                  <Field
                    fullWidth
                    component={RFTextField}
                    disabled={submitting || sent}
                    required
                    name="password"
                    autoComplete="new-password"
                    label="비밀번호 확인"
                    type="password"
                    margin="normal"
                  />
                </Grid>
                <Grid item xs={12} sm={12}>
                  <Field
                    autoFocus
                    component={RFTextField}
                    disabled={submitting || sent}
                    autoComplete="phonenum"
                    fullWidth
                    label="전화번호"
                    name="phonenum"
                    margin="normal"
                    required
                  />
                </Grid>
              </Grid>

              <FormSpy subscription={{ submitError: true }}>
                {({ submitError }) =>
                  submitError ? (
                    <FormFeedback error sx={{ mt: 2 }}>
                      {submitError}
                    </FormFeedback>
                  ) : null
                }
              </FormSpy>
              <FormButton
                sx={{ mt: 3, mb: 2, backgroundColor: "main.aa"}}
                disabled={submitting || sent}
                fullWidth
              >
                {submitting || sent ? "In progress…" : "Sign Up"}
              </FormButton>
            </Box>
          )}
        </Form>
      </AppForm>
      <AppFooter />
    </React.Fragment>
  );
}

export default withRoot(SignUp);
