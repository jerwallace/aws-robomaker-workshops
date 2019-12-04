---
title: "Workshop Clean-up"
chapter: true
weight: 12
description: Please read through and complete each activity before starting the next. If something doesn't look correct, ask for assistance as we want to make sure the concept covered are well understood. Also, if you find a bug - submit a pull request!
---

# Workshop cleanup

If you used a workshop code as part of an Official AWS Event (like re:Invent), please disregard below. Your resources will automatically be removed.

AWS only charges for consumed resources. Please follow the steps below to shutdown or delete resources so you will not be charged.

1. Delete the **S3 bucket** by selecting the bucket then clicking *Delete* from above the list of buckets.

2. From the **AWS RoboMaker console**, make sure there are no simulation jobs in progress. If there are, select them and click *Actions->Cancel*.

3. Also from the **AWS RoboMaker Console**, from Development->Development environments, click on the environment name, *Edit*, then *Delete* from the AWS Cloud9 console.

4. In the **CloudWatch Console**, under *Logs*, select each LogGroup (`/aws/robomaker/SimulationJobs` and `jetbot`) and click *Actions->Delete log group*.

5. In **CloudFormation**, select the stacks you created and click **Delete Stack**.