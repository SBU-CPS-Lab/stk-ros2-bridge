# Commit map: message cleanup of 2026-10-19

On 2026-10-19 the message of one commit on `perf/e6-throughput` was
reworded (old ff43667, a runbook note about keeping other programs off
the working tree). Rewording it changed the hash of that commit and of
every commit after it. Nothing else changed: for every commit below the
tree hash, the author, the committer and both dates are identical
before and after, and every other message is unchanged byte for byte.
Commits older than old ff43667 kept their hashes. `main` is unchanged.

**Results carry the old hashes.** Folder names under `results\` and the
`code_version` in every record were written before the rewrite and are
not changed:

| Final runs | Old hash (in folder names and records) | New hash | Tree (identical) |
|---|---|---|---|
| E1, E3, E6 | db7cfb6 | 6c32287 | 8465395f02ff6c228d5ce9da04614d2fb46d853c |
| E2 | 91f36b9 | 81bc202 | 4a5687f13a3c3dde78019b649b834f328a4c8f48 |

Hashes cited in the docs before this rewrite are old hashes; look
them up here. The final-run hashes are written as "new (old)" where
the docs cite them.

Every commit on `main..perf/e6-throughput`, oldest first:

| # | Old | New | Tree | Subject |
|---:|---|---|---|---|
| 1 | 1c6f90e | 1c6f90e (unchanged) | 935deba35a42d821f3c0bc41edf435252e8519dd | fix(acav): prepare standalone run configuration before bridge startup |
| 2 | 69f4f7e | 69f4f7e (unchanged) | f6b711464092403120b38c96b6dc272d89742881 | fix(acav): warn when an existing run config has no run_id |
| 3 | cb48b21 | cb48b21 (unchanged) | fb701b538bcdafc0081449d98f41d1cbb588a467 | docs(dev): phase 1 E6 impact analysis of combined attitude and orbit batching |
| 4 | bf6a74d | bf6a74d (unchanged) | f95f31377fbae6ebc56b734a9802da6e768fe716 | feat(acav): add read-only HPOP force-model inspection script |
| 5 | 183645d | 183645d (unchanged) | 1be59cd6b2011cfdddc89063826c9866750ec1f4 | feat(bridge): add opt-in sensing equivalence diagnostic |
| 6 | 9abee01 | 9abee01 (unchanged) | 6ada13a06a82312847ced583d60e876472325d39 | docs(dev): record the E6-diag-1 live results in the phase 1 analysis |
| 7 | f9d6aa2 | f9d6aa2 (unchanged) | 73097390a9ad4d00eaaf1dbfee39d8d56e1343de | perf(bridge): share validated attitude and angular-rate provider reads |
| 8 | ef9079a | ef9079a (unchanged) | b16c75b79bcee33f5a0c70f65646dbe5857b3d4d | test(bridge): replay the lockstep sequence and log a combined-read summary |
| 9 | 6c9059b | 6c9059b (unchanged) | 7de67bed0c117481872f59510cf13f7d27c18f8e | perf(bridge): batch HPOP orbit reads with exact-grid validation |
| 10 | a4412a9 | a4412a9 (unchanged) | 25312a86a08f04bd77c89f6d5f53fbd7d0a48a6b | docs(dev): record three more pre-existing issues in the phase 1 analysis |
| 11 | 7e3b111 | 7e3b111 (unchanged) | b216b867564e8686e518c0bc6419e586ee5cac74 | fix(bridge): emit sensing summaries before the bridge is terminated |
| 12 | eca37cf | eca37cf (unchanged) | 60b16f849c7deae17db37195f61b387d21d71836 | docs(dev): record the E6-BC-1 live results in the phase 1 analysis |
| 13 | cff9257 | cff9257 (unchanged) | 89f0b8313e89b0505d0e83c56048c7bb0173be25 | docs(dev): record the Phase 1 final results (provisional timings) |
| 14 | bf283ad | bf283ad (unchanged) | 88e703d91c52f16ee797cc41c06a2d3e09b3b530 | docs(runbook): add running validation sessions safely |
| 15 | 88a6c2c | 88a6c2c (unchanged) | 55fa7547ca2b6ae01fd276c97ccde0dc9763c53e | docs(dev): add the commit map for the rewritten branch messages |
| 16 | 13f1c48 | 13f1c48 (unchanged) | 35817d02efb38be0eb5debeca03c904bdf100f2b | feat(acav): refuse to resume a campaign over records from other code |
| 17 | ac9a629 | ac9a629 (unchanged) | ba2922613195962cb3fa4a50b6968bfc38e6bb33 | feat(acav): refuse to start a campaign from modified code |
| 18 | 0d99f09 | 0d99f09 (unchanged) | c0c1f083a0f886758df825dbf56244dad2236540 | fix(acav): move a previous attempt aside before re-running a run |
| 19 | d25eba5 | d25eba5 (unchanged) | ac40fbde1376c94736e83f5bfa42b0e30f3977c7 | perf(bridge): time the STK clock advance where it executes |
| 20 | 8f8e038 | 8f8e038 (unchanged) | 6174fd2794cb02c751a7fc28d9b2fdba3fdf0d4a | feat(acav): add --diagnostic-label so report never pools one-off runs |
| 21 | ff43667 | 31ae357 | f34f797d7e525e6e435b0264e27b2968601d9535 | docs(runbook): keep other programs and editors off the working tree (message reworded) |
| 22 | c6dceb7 | e9d8b1e | aac058affc6d1481d395e9affdc485c0a0defbf8 | feat(twin): fingerprint the twin's code and digest its run config |
| 23 | 613f094 | 148d779 | aec472b6c60b05ca5f0cdc668b036329cc2ef3ca | feat(twin): publish which run config and code the twin runs |
| 24 | f2f5746 | a408dd4 | 7dc1b99e9e0799f55c046455c68c4b71aa25daee | feat(acav): start the scheduler only for a twin that loaded this run |
| 25 | c367eed | 8d6fffe | f0894bf714f36b1481c5564e17c2fa0cc5fa7661 | docs: record the twin identity session and the STK graphics result |
| 26 | 883c336 | e571ad9 | e7f998990b6d2b69d8386b65fea4ead7798dca3c | fix(acav): record an idle-check refusal as a failed step |
| 27 | 070be93 | be1e1a2 | 8249cefd24e3790344035497f2038b33fe4c2ea9 | feat(bridge): record STK's graphics state during a run |
| 28 | 9dc9d41 | c55fb47 | d3cf990be0900da35af44c9228e467839a6bea7f | feat(acav): require a graphics mode for every run and record it |
| 29 | 00719b3 | 7a9c6a4 | ca10f547423c7987dd46fdfff7a0b2b40c625d87 | feat(acav): refuse to report records from different graphics modes |
| 30 | d02d2c5 | 2b145d0 | 38ca8d4a1873fc89544088763c0d4ff7603460e8 | feat(acav): refuse to report mixed or unverified graphics runs |
| 31 | ebc23c7 | 20fe373 | 431344bb52b2bc80241075a48e2ec9a9ad30261f | docs(dev): record the Phase 3 backlog with a feasibility outline |
| 32 | 3d52f0b | 6d286d6 | 017690393cd92ef5317125c7a68957d484f9477a | docs(dev): correct two line references in the Phase 3 backlog |
| 33 | ab7c4d2 | 544a41e | 8f13e964ba7e311f3a82e10bcf724e3d865977e8 | fix(acav): apply the requested execution mode to the bridge and twin |
| 34 | 413d540 | 4bbc277 | 4c88186d7d03e3a13c9b436a31981eea855bb90e | fix(bridge): take the summary and graphics checkpoints in free-run too |
| 35 | 7379995 | a949f83 | c239fcfd4f37dcd517568fd25234f1fb196ec30b | feat(twin): integrate free-run samples with zero-order-hold sub-steps |
| 36 | cde4d4e | 796071a | 837febeb24f79c40f48f8fecf93fa125840e030a | feat(acav): record a free-run twin's sub-step statistics |
| 37 | c61fbd2 | 8905203 | 0d47ca48fa9221fc499f38a713bf07179d00dde8 | docs(runbook): describe E3 free-run runs |
| 38 | 8caa919 | 85e8b4a | e804ec3472314a0f073ab1e4725146a58b9179f9 | feat(bridge): hold the free-run clock at t = 0 until participants ack |
| 39 | 48813ac | efd8888 | e7b44f2dc4cb264fa978ac6453baf81e55d9a818 | feat(acav): record the free-run start gate and require it to open |
| 40 | a6436a4 | 7b205a3 | 0c9229fc9672bec5f2633153393c0f01e8003eab | fix(bridge): pace free-run cycles from the cycle start |
| 41 | 9cbc0f7 | e561f80 | 7df6de8135ef3b0810c71aab1341ba3a628e940a | feat(acav): record and enforce the real-time factor |
| 42 | 1b561a5 | 63dcaf5 | 4195a87e0da3e12258171f5d61ff6ac2a940e4b2 | feat(acav): offline free-run timing and coupling analysis |
| 43 | e45a8a1 | 96f4c9e | e1851c4212ea5666db06c06915fed62ec88743f8 | docs: record the E3 free-run pilot and the issues it surfaced |
| 44 | 72be372 | 71a2a2d | 3d25b9a1528f8b3d5df90f87a96ef0be44879f41 | fix(twin): propagate reference geometry to each free-run sub-step time |
| 45 | 168cf19 | 2c7786c | 1a144a08252bd996fb33b620a67a4cf536cc3d6b | fix(acav): never drop an unscored target from the run record |
| 46 | a0badb3 | afe7a0d | b3ab8887d97f7674c6fe57d0c2594be0e3d67371 | docs: backlog items from the 72be372 live session |
| 47 | 1816a36 | a966ac0 | 577e98e398bedf43cef2609573decc8a74fedb8c | fix(acav): refuse records whose targets do not cover the scenario |
| 48 | 88e60f1 | 426ac93 | 4eaa4573b8bb56d38b52d1d990ecee6474106f11 | fix(acav): compute access only after an attitude sample at or after the image epoch is written |
| 49 | 4b26de0 | 8a3289b | c920f26b4400ce607f93266fb2136c1dd2946411 | perf(twin): pace the Simulink loop and publish the twin identity on change |
| 50 | a6b31d1 | f2e3eaf | 7ceb1f06ee3eccf7002ab3d3c22be64977838771 | feat(acav): report the twin's achieved loop rate in the coupling analysis |
| 51 | 1628866 | a22bd3d | 2164362426406b22a4ad5e79d5bce4d3597fe290 | fix(bridge): publish ScenarioTime after sensing in free-run |
| 52 | 8345551 | b9b4b86 | 78adeecc7ddc638065ea15b10d13ba554ec9b3a9 | feat(twin): log the free-run loop counters to twin_loop.csv |
| 53 | 567ec78 | db933fb | cf56c8de0be26a52e48be11ca5ffff1bcad1c070 | docs: record the free-run fixes and their live results |
| 54 | a9bf415 | 3aeabb8 | fcdca4da8da3028e938846eb46b88d3eccecb126 | fix(acav): line-ending-independent code fingerprint and scenario digest |
| 55 | 880171c | e4d10aa | 74d964cb5f96514383c933111cdb9b637197afcf | feat(acav): let the runner start and stop the MATLAB twin |
| 56 | 9fed085 | f0dcffc | 8f30ad53e440e2f839ddb7560b8ad669d5658cec | feat(acav): run campaigns with the automated twin launch |
| 57 | 2ca0ba4 | 9b6e0dc | 7ca02e3bc5850f5cc2d2b2d54a477747379a13a8 | fix(acav): never let the console or a post-run step hold a run |
| 58 | b394945 | d677fee | fe0eb0d393aac1687d35eb954339300ef2b79961 | feat(acav): build and verify any scenario file; ground station optional |
| 59 | 96a1c6d | ac8dfbd | 2db36c39c901a01923ae865a69bcdb707949cacc | feat(acav): seeded imaging-target scenario generator |
| 60 | 9afee2c | e43f1c3 | c7c6594d73a1e913fdaa6410b768425ee7c41613 | feat(acav): thread generated scenarios through runs and reports |
| 61 | 29a3cae | ca0b74c | 3cd497c084dbf5868ea0ce8dc1df568fa2014276 | feat(acav): scenario generator version 2 fills the window |
| 62 | d97800c | 0c29ba9 | b045a49701c4ac7037950b3b5576d639e4efecd8 | feat(twin): report what happened between two TwinStatus messages |
| 63 | 24512a4 | f3ddc9b | bd1dd5e799a2388494cce308081605e589e1240a | fix(twin): record why the Simulink twin stopped by itself |
| 64 | 3186642 | cd2e65c | 360103e4736ef1d1a3b82176e54ddfddbfe1ee32 | fix(twin): return saturationEvents as uint32, the class the model declares |
| 65 | f19ced4 | 8055934 | b852feacdff829872811abc090282f7e8f07275c | fix(run): give each ROS wait its own context and keep its error |
| 66 | 3d143bc | 78d45c4 | b35e406e67fef717c5841855bb246ab0eccecbbe | feat(bridge): time each startup phase; the runner waits 180 s and records it |
| 67 | 24b5eeb | b96285f | e84fadcb99a0f2631b6489c514700a0635ecde1b | feat(run): attach to the operator's STK only, and refuse one started through COM |
| 68 | c8a0f44 | ed15f8f | db962e852c83c65675448f757638b0482229dbd1 | docs(bridge): the startup-order note names the early status publisher |
| 69 | 7bb1087 | 4916c22 | 119ca6af54d5eb2e2fecb4cf6eeb00d43057e1ba | docs: describe the framework as it is now |
| 70 | 35c6a16 | 6658de7 | c86048fb46076b93649d5571269ea19c354a6aab | docs(acav): describe the case study as it is now |
| 71 | 1ac5dd2 | b65ed8d | 68b343231d81227fe99a3879a553951c18ed98e2 | docs(runbook): rewrite as an operator manual |
| 72 | 3fa63c4 | 461ea5d | 90d06db7bd62b4a1ed91bd8526d58af3742837ad | docs: current evidence and a documentation map |
| 73 | b76000e | dfba6eb | dc84ffed0b296abcb4b196c14d5d3eda111b8b3c | test(run): the twin-ack wait must stop when MATLAB exits |
| 74 | 9eaa451 | 04ce03b | 241710563661c7436936877c16ad24e51c320af0 | test(run): fake the idle check and topic capture in the twin-ack tests |
| 75 | 3be3086 | 11fc638 | d38062fcd00ca841ee30941234160ae33dbe2d33 | fix(run): stop the twin-ack wait as soon as MATLAB exits |
| 76 | 695e627 | 40601a6 | 1a9b1691609d41dd8934c278dc935ebf74533adb | test(campaign): E4 is removed; old E4 and E5 records still load |
| 77 | bf0d4fc | 30e352b | ddc6dac0df0a48125e93af5432df51b95ce47d4a | refactor(campaign): remove E4, the dropped controller gain sweep |
| 78 | b795a6b | 54436ae | 07eefbcb1921ae368aef3e51bb6f90960ed3b64a | docs(runbook): name no editor products in the session checklist |
| 79 | c3cb8bd | 60fd101 | d6f6e4eb582fe514def7b95f5c456667f3af04a1 | test(analysis): exact paired statistics and a paired E2 report |
| 80 | 2dc7c9e | 8abf2cd | b9932473a02c9f9c6e236cb0e5cd94ee1ab5290b | test(analysis): give the paired-report fixtures a valid twin identity |
| 81 | 20e1f54 | 7c19011 | 5f09ac0403dbbb9552d2cd8a9f44745e6a745e68 | feat(analysis): analyse E2 across seeded scenarios as pairs |
| 82 | d065fa4 | 1bc74f3 | df69504d6bf92ff84e6dd61121bae1434bd5b3c9 | docs(analysis): where the paired E2 analysis is described |
| 83 | 8978c00 | 370728a | c45632b3660700d0408641f9ccfdebbf7ca42564 | test(e1): six interface-mismatch types, injected at the publish side |
| 84 | 53a72bf | 9fe3a3e | 1db61eda2fa4b61e270e3e82e060c5375a0c9d62 | feat(e1): six interface-mismatch types; five injected at the publish side |
| 85 | 88bb862 | fce9821 | d640c88c05e04e2cab37f82741c7e360eac35c9f | docs(e1): six mismatch types, their injection points and the run formula |
| 86 | a911355 | abf9f06 | 4518e2e1bd89e0de09ef86bdc47072dcf9243eb4 | test(e1): label a time_offset fail-stop confirmed only on its own evidence |
| 87 | 7adf356 | 5791484 | c139db2e291b46d8d5dbed129dfceb4abd174e72 | feat(e1): confirm a time_offset fail-stop only on the run's own records |
| 88 | bf6e6d0 | 69ee022 | a80712b94cf4f0f1fed5aece2eb903f9dddcb77a | test(e3): one matrix of mode x fault x repeats, every run launchable |
| 89 | 7675185 | a682c12 | 3b3ba9dc6dbf6d0dd2688fdedd40007ecd7a6b34 | feat(e3): one matrix of mode x fault x repeats, with committed launch commands |
| 90 | 79befa0 | 9062caf | 252d7f4d2cf2d8c87378aa8f98f5407ce1e80441 | docs(e3): one matrix campaign, fault replay included, with its commands |
| 91 | 7104639 | 208804c | b8eaa3e23e44418c373c8d1f3b5381c518cf0846 | test(e3): an explicit fault-seed list; the old comparison labelled |
| 92 | 46365a5 | c20e12f | 04710ad4af10d2c614093a2e9eb966e8aa5e0725 | feat(e3): a configurable fault-seed list; the old comparison labelled |
| 93 | d0bf796 | f035029 | f36cdfefcde0e95ed3d15242624ac0801607a62f | docs: the live verification session, and what is left after it |
| 94 | 88f90b9 | be57b29 | 52a1baa292950fdbaf796f484d179ef032bae654 | test(twin): every path handed to MATLAB must be absolute |
| 95 | 6afe951 | 62a3211 | a934d2c1829c47ee0fb792272fffee512526d550 | fix(twin): hand MATLAB absolute paths only |
| 96 | 40d97dc | d06491e | ccbf8a688005faa978e7a9f2e16b54f33fb5e58c | test(e1): classify a run without a record honestly |
| 97 | 5e0d0f3 | 22f9c39 | 940daae0a9c1b331414fadaeb0414235e9e8849b | feat(e1): report what a run without a record actually was |
| 98 | 845870a | 1679f14 | 36a1996a3ee0a8105d1ec404ecae4922efa00be8 | test(run): record leftover twin processes; wait out or refuse a ROS server |
| 99 | 01bf18d | 8d910ab | 79ce64923a1a4e0b11cf7ffd1b056c8d21670cf6 | feat(run): record leftover twin processes before MATLAB starts |
| 100 | dd5daaa | 899c627 | 167b5974401d67284c810f894fc9592b9a1b0055 | test(e3): the plan uses the measured rtf 0.5 wall time |
| 101 | 57dbbfd | 7ba175e | 6101e50116948b3f4fd60dc754def10b6c5c68f2 | feat(e3): the plan uses the measured rtf 0.5 wall time |
| 102 | f55d08b | 95df75e | f63f78ce7c16f750c494a15b3894ff563d1e130b | docs: absolute output folders, and the live verification session's outcome |
| 103 | 4972adb | 69ef571 | 30f2a78fbf45efb6b74dae46868ffbeb8ad08946 | test(e3): repeats per execution mode (--repeats-rtf05, --repeats-lockstep) |
| 104 | 56f2e18 | 3b236ab | f89a248afb80bde567d81560d8009212d66f9920 | feat(e3): repeats per execution mode |
| 105 | db7cfb6 | 6c32287 | 8465395f02ff6c228d5ce9da04614d2fb46d853c | docs: final-run section, limitations, and the old issues at the freeze |
| 106 | 89c3c7e | 2bea0b9 | 6045dafde704ccfa6f3b7cd03279bdba96356625 | test(build): seeded scenarios one after another, and E2 over them |
| 107 | 8e416e4 | 47a5b02 | 72e5d9327972a9daa0cb615d2596a039256573db | feat(build): seeded scenarios one after another, and E2 over them |
| 108 | eb69251 | 8357763 | 1d66e19afd81016ff66536f36fc70718bd210358 | docs: E2 final runs in one command (run-seeds); the build leaves graphics off |
| 109 | d52b730 | f222f33 | 433eddc261c17954dae826d717e56583d942e3cb | test(e2): seeded scenarios resolved under the workspace src tree, checked before any build |
| 110 | 499e4bc | 936f4c1 | 03fb0f72841df38de1828be0ddca2b0c37126dde | fix(e2): seeded scenarios from the workspace src tree, checked before any build |
| 111 | 91f36b9 | 81bc202 | 4a5687f13a3c3dde78019b649b834f328a4c8f48 | test(e2): run-seeds fakes name their scenario folder, independent of the current directory |
| 112 | 2e6be6e | 81bd93e | 12960927ab6d2232a2a4a54e7380377e897bb151 | test(e2): report finds seeded scenarios under the workspace src tree |
| 113 | 53004a3 | 2244a06 | 77b0f600e12161252bf16686b12ec8c1e0da435a | fix(report): seeded scenarios found under the workspace src tree |
| 114 | 2aeb988 | 72bad2b | 201603f147ac77da37b610b30fca704a515ed87f | test(figures): the E2 paper figure script writes its three figures |
| 115 | c78a6de | 7c362e3 | 31abd8e1e40caf4e8facfd65387dcb847b7e03bd | feat(figures): the E2 paper figure script, copied unchanged from the final E2 folder |
| 116 | 7a5f438 | 1bb3df0 | c1a7517922b35c6df1fedf4f6d3a2cf96977e124 | docs: final-run state, run-seeds and report lookup, rebuild after code changes, archived pilot paths |

Commits made after this map (this file and later) exist only with
new hashes.
