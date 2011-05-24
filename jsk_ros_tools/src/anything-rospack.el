;; anything-rospack.el
;;
;; this package depends on anything.el
;; You can get anything.el from
;; http://www.emacswiki.org/cgi-bin/wiki/Anything
;;
;;
;; I recomend to use like below:
;; (setq anything-sources
;;      (list anything-c-source-buffers
;;            anything-c-source-file-name-history
;;            anything-c-source-imenu
;;            anything-c-source-recentf
;;            ;;anything-c-source-man-pages
;;            ;;anything-c-source-info-pages
;;            anything-c-source-calculation-result
;;            anything-c-source-kill-ring
;;            ;;anything-c-source-bookmarks
;;            anything-c-source-locate
;;            anything-c-rospack-source))
;;
;; anything-rospack.el is writteny by R.Ueda
;;  contact: ueda@jsk.t.u-tokyo.ac.jp

(require 'anything)

(defvar anything-c-rospack-source
  '((name . "rospack list")
    (candidates . (lambda ()
                    (delete "" (split-string
                                (shell-command-to-string "rospack list")
                                "\n"))))
    (action . (("Select" . (lambda (x)
                             (find-file (cadr (split-string x " ")))))))))

;; 

(provide 'anything-rospack)
