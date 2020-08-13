---
title: "Prep: Workshop Setup"
chapter: true
weight: 1
description: "We will start by setting up your AWS account to develop robot applications with AWS RoboMaker."
---

# Workshop Setup

Before we get started learning about robot development, we need to configure a few things.  This workshop can be completed in two ways:  at a public event where AWS provides temporary AWS accounts; or on your own, where you use your own AWS account.

Are you completing this workshop at a public event or training session, **and** have you have been provided with a 12-character workshop code?


   <button class="ui-button" onclick="document.getElementById('with_code').style.display = 'block';document.getElementById('no_code').style.display = 'none';">**Yes, I have a code**</button>&nbsp;&nbsp;<button class="ui-button" onclick="document.getElementById('with_code').style.display = 'none';document.getElementById('no_code').style.display = 'block';">**No.  I will use my own AWS account.**</button>

<div id="with_code" style="display:none">

{{% md %}}
### Log  in to the AWS Console and set the AWS Region

For this workshop, we've created temporary AWS accounts for all attendees.  You were provided with a code to access your AWS account for the workshop.  You will need that code in the next steps.  To get started, enter the AWS Console by going to this web site:

**[https://dashboard.eventengine.run](https://dashboard.eventengine.run)**   

On the *Who are you?* form, enter the code you were provided (ensure the case is correct) and click **Proceed**.

![Event Engine Login](../../images/event-engine-login.jpg)

Then click on the **AWS Console** button, and then the **Open Console** button on the pop-up.

**Congratulations!** You have completed the setup portion of the workshop.

**[Continue to the next module.](../develop/)**
{{% /md %}}
</div>

<div id="no_code" style="display:none">
{{% md %}}
### Log  in to the AWS Console and set the AWS Region

When using your own AWS account to complete this workshop, your user needs read and write permissions to several services, including AWS RoboMaker, Amazon S3, AWS Cloud9, Amazon CloudWatch, Amazon VPC, AWS Lambda and AWS Identity and Access Management.

Login to your AWS console by accessing https://console.aws.amazon.com and set your region to US West (us-west-2) Oregon.

**Congratulations!** You have completed the setup portion of the workshop.

**[Continue to the next module.](../develop/)**
{{% /md %}}

</div>
