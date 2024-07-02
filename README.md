# AE-ORB-SLAM
\cite{zarringhalam2022selfsupervisedvectorquantizationvisualslam}
To use the MATLAB package, MonocularVisualSimultaneousLocalizationAndMappingExample from vision package, for ORB-SLAM, please type the following command in MATLAB console:
openExample('vision/MonocularVisualSimultaneousLocalizationAndMappingExample').
There are totally three files that need modification to run AE-ORB-SLAM. The main file is directly imported here from matlab website:

https://www.mathworks.com/help/vision/ug/monocular-visual-simultaneous-localization-and-mapping.html

the bagOfFeatures method should be adjusted to accept a proper helperExtractorFunction to accept only ORB features.


@misc{zarringhalam2022selfsupervisedvectorquantizationvisualslam,
      title={Self-supervised Vector-Quantization in Visual SLAM using Deep Convolutional Autoencoders}, 
      author={Amir Zarringhalam and Saeed Shiry Ghidary and Ali Mohades Khorasani},
      year={2022},
      eprint={2207.06732},
      archivePrefix={arXiv},
      primaryClass={cs.RO},
      url={https://arxiv.org/abs/2207.06732}, 
}
