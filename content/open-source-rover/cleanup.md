---
title: "Workshop Clean-up"
chapter: true
weight: 15
description: "At the end of the session, considering cleaning up the resources that were created.  AWS only charges for consumed resources."
---

# Workshop cleanup

You only need to follow these steps if you've completed this workshop in your own AWS account, and you want to remove resources to prevent any additional AWS charges.

If you're completing this workshop at an AWS event such as a conference or training session, and you used a 12-character code to log into an account that was provided as part of the workshop, you can stop now.  There is no need to complete the steps below.

AWS only charges for consumed resources. Please follow the steps below to shutdown or delete resources so you will not be charged.  This step is optional.

1. Delete the **S3 bucket** by selecting the bucket then clicking *Delete* from above the list of buckets.

2. From the **AWS RoboMaker console**, make sure there are no simulation jobs in progress. If there are, select them and click *Actions->Cancel*.

3. Also from the **AWS RoboMaker Console**, from Development->Development environments, click on the environment name, *Edit*, then *Delete* from the AWS Cloud9 console.

4. In **CloudFormation**, select the stack you created and click **Delete Stack**.